#include <vector>
#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "geometry/image_cluster.h"

#include "openMVG/matching/indMatch_utils.hpp"
#include "stlplus3/file_system.hpp"

using namespace std;
using namespace openMVG;
using namespace openMVG::matching;
using namespace GraphSfM::geometry;

// #define __DEBUG__

DEFINE_string(abs_imglist_path, "", "The file that stores all the path of images");
DEFINE_string(weight_file, "", "The file the stores the weight between two image nodes");
DEFINE_string(cluster_option, "naive", 
             "The option that decide to use naive image cluster or expansion image cluster");
DEFINE_int32(max_cluster_size, 100, "The maximum capacity of a cluster(the maximum number of images)");
DEFINE_double(completeness_ratio, 0.7, "The ratio that measure the repeateness of adjacent clusters");
DEFINE_string(output_dir, "", "The directory that stores the partition result");
DEFINE_bool(copy_images, false, "Whether to copy images into the new sub-folders");
DEFINE_string(matches_dir, "", "The directory that stores the matches file in openMVG format");

/// Export vector of IndMatch to a stream
static bool PairedIndMatchToStream(const PairWiseMatches& map_indexedMatches,
                                   std::ostream& os)
{
    for (auto it = map_indexedMatches.begin(); it != map_indexedMatches.end(); ++it) {
        const size_t i = it->first.first;
        const size_t j = it->first.second;
        const std::vector<IndMatch> & vec_matches = it->second;
        os << i << " " << j << '\n' << vec_matches.size() << '\n';
        copy(vec_matches.begin(), vec_matches.end(),
             std::ostream_iterator<IndMatch>(os, "\n"));
    }
    return os.good();
}

vector<PairWiseMatches> BuildClusterMatches(const PairWiseMatches& mapMatches,
                                            const vector<shared_ptr<ImageGraph>>& igList)
{
    vector<PairWiseMatches> matches_list;
    map<Pair, IndMatches>::iterator ite;
    
    for (auto ig : igList) {
        PairWiseMatches pm_matches;
        // std::vector<EdgeMap> adj_maps = ig->SerializeEdges();
        auto adj_maps = ig->GetEdges();
        // std::vector<ImageNode> nodes = ig->SerializeNodes();
        auto nodes = ig->GetNodes();
        for (auto em_it = adj_maps.begin(); em_it != adj_maps.end(); ++em_it) {
            auto edge_map = em_it->second;
            for (auto ite = edge_map.begin(); ite != edge_map.end(); ++ite) {
                // Pair pair(ite->second.src, ite->second.dst);
                int src = nodes[ite->second.src].id;
                int dst = nodes[ite->second.dst].id;
                Pair pair1(src, dst), pair2(dst, src);
                auto match_ite1 = mapMatches.find(pair1);
                auto match_ite2 = mapMatches.find(pair2);
                if (match_ite1 != mapMatches.end()) {
                    IndMatches ind_matches = match_ite1->second;
                    pm_matches.insert(make_pair(pair1, ind_matches));
                } else if (match_ite2 != mapMatches.end()) {
                    IndMatches ind_matches = match_ite2->second;
                    pm_matches.insert(make_pair(pair2, ind_matches));
                }
            }
        }
        matches_list.push_back(pm_matches);
    }
    return matches_list;
}

void OutputClusterMatches(const vector<PairWiseMatches>& matchesList, string dir)
{
    // if (!stlplus::folder_exists(dir)) stlplus::folder_create(dir);
    ofstream stream(dir + "/matches_list.txt");
    if (!stream.is_open()) {
        LOG(ERROR) << "matches list file " << dir + "/matches_list.txt cannot be opened!";
        return;
    }
    for(int i = 0; i < matchesList.size(); i++) {
        string matches_name = dir + "/cluster_matches" + to_string(i) + ".txt";
        ofstream fis(matches_name);
        stream << matches_name << endl;
        if(fis.is_open()) PairedIndMatchToStream(matchesList[i], fis);
        fis.close();
    }
    stream << endl;
    stream.close();
    LOG(INFO) << "matches_list file successfully saved in "
              << dir + "/matches_list.txt"; 
}

PairWiseMatches RecoverMatches(string fileName)
{
    PairWiseMatches map_GeometricMatches;
    ifstream fis(fileName.c_str());
    IndexT i, j;
    int matchesSize;
    int indLeft, indRight;
    if(fis.is_open()) {
        while(fis >> i >> j) {
            fis >> matchesSize;
            IndMatches indMatches;
            while(matchesSize--) {
                fis >> indLeft >> indRight;
                indMatches.push_back(IndMatch(indLeft, indRight));
            }
            map_GeometricMatches.insert(make_pair(Pair(i, j), indMatches));
        }
    } else {
        cout << "Failed to open file " << fileName << " in recoverMatches" << endl;
        return map_GeometricMatches;
    }
    return map_GeometricMatches;
}


int main(int argc, char ** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    if(argc < 7) {
        cout << "Please input file path of the vocabulary tree\n" << endl;
        cout << "Usage: \n" << 
            "i23dSFM_GraphCluster absolut_img_path absolut_voc_path " << 
            "cluster_option=expansion max_img_size completeness_ratio output_dir\n";
        cout << "Notice: cluster_option must be 'naive' or 'expansion'\n";
        return 0;
    }

    // It's best to put them on image root path
    string img_list = FLAGS_abs_imglist_path;
    string voc_file = FLAGS_weight_file;
    string cluster_option = FLAGS_cluster_option;
    int max_cluster_size = FLAGS_max_cluster_size;
    double completeness_ratio = FLAGS_completeness_ratio;
    string matches_dir = FLAGS_matches_dir;
    string output_dir = FLAGS_output_dir;
    bool copy_images = FLAGS_copy_images;
    float relax_ratio = 0.35;

    ImageClusterOption image_cluster_option((size_t)max_cluster_size, completeness_ratio, 
                                            relax_ratio, copy_images);

    ImageCluster graph_cluster(image_cluster_option);
    
    string dir = stlplus::folder_part(voc_file);
    ImageGraph img_graph = graph_cluster.BuildGraph(img_list, voc_file);

#ifdef __DEBUG__
    img_graph.ShowInfo();
#endif

    size_t clust_num = 1;
    LOG(INFO) << "nodes: " << img_graph.GetNodesNum();
    LOG(INFO) << "cluster capacity: " << image_cluster_option.cluster_upper_size;
    if(img_graph.GetNodesNum() < image_cluster_option.cluster_upper_size) {
        LOG(ERROR) << "size of graphs less than cluster size, camera cluster is the origin one";
        return 0;
    } else {
        // clust_num = img_graph.GetNodesNum() / image_cluster_option.cluster_upper_size;
        clust_num = 2;
    }

    std::unordered_map<int, int> cluster_map = img_graph.NormalizedCut(clust_num);
    queue<shared_ptr<ImageGraph>> sub_image_graphs = 
        graph_cluster.ConstructSubGraphs(img_graph, cluster_map, clust_num);
    graph_cluster.CollectDiscardedEdges(img_graph, sub_image_graphs);

    if(cluster_option == "naive") {
        graph_cluster.NaiveImageCluster(sub_image_graphs, dir, clust_num);
    } else if(cluster_option == "expansion") {
        vector<shared_ptr<ImageGraph>> insize_graphs = 
            // graph_cluster.ExpanImageCluster(img_graph, sub_image_graphs, dir, clust_num);
            graph_cluster.ExpanImageCluster(img_graph, sub_image_graphs);
        // graph_cluster.MoveImages(insize_graphs, dir);
        graph_cluster.MoveImages(insize_graphs, dir, output_dir);
        
        if (!matches_dir.empty()) {
            PairWiseMatches pairwise_matches = RecoverMatches(matches_dir + "/matches.f.txt");
            vector<PairWiseMatches> matches = BuildClusterMatches(pairwise_matches, insize_graphs);
            OutputClusterMatches(matches, stlplus::folder_part(img_list));        
        } else {
            LOG(WARNING) << "matches directory empty!";
        }
    } else {
        LOG(ERROR) << "cluster_option must be 'naive' or 'expansion'\n";
        return 0;
    }

}