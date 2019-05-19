#include "image_cluster.h"

#include "stlplus3/file_system.hpp"
#include "omp.h"

#include <glog/logging.h>

#define OUTPUT_DISCARDED_EDGES

namespace GraphSfM {
namespace geometry {

ImageCluster::ImageCluster(size_t upper, float cr)
{
    _cluster_option.cluster_upper_size = upper;
    _cluster_option.completeness_ratio = cr;
}

ImageCluster::ImageCluster(const ImageClusterOption& cluster_option)
{
    _cluster_option = cluster_option;
}

void ImageCluster::SetClusterOption(const ImageClusterOption& cluster_option)
{
    _cluster_option = cluster_option;
}

ImageGraph ImageCluster::BuildGraph(const string& image_list, const string& voc_file) const
{
    ImageGraph image_graph;
    ifstream img_in(image_list);
    ifstream voc_in(voc_file);
    string img;
    int idx = 0;
    
    if(!img_in.is_open()) {
        LOG(ERROR) << "File of image list cannot be opened!";
        return image_graph;
    }
    while(img_in >> img) {
        graph::ImageNode inode(idx, img);
        image_graph.AddNode(inode);
        idx++;
    }
    img_in.close();

    if(!voc_in.is_open()) {
        LOG(ERROR) << "File of vocabulary tree cannot be opened!";
        return image_graph;
    }
    size_t src, dst;
    float score;
    int k = 0;
    while(voc_in >> src >> dst >> score) {
        // an undirect weight graph is required
         if(src != dst) {
            graph::Edge edge(src, dst, score);
            image_graph.AddEdge(edge);
         }
    }
    voc_in.close();

    LOG(INFO) << "Node size(Before remove singleton nodes): " << image_graph.GetSize();
    image_graph.CountInDegrees();
    image_graph.CountOutDegrees();
    image_graph.CountDegrees();
    image_graph.RemoveSingletonNodes();
    LOG(INFO) << "Node size(After remove singleton nodes): " << image_graph.GetSize();

    return image_graph;
}


void ImageCluster::MoveImages(queue<shared_ptr<ImageGraph>>& image_graphs, const string& dir)
{
    int i = 0;
    while(!image_graphs.empty()) {
        shared_ptr<ImageGraph> ig = image_graphs.front(); image_graphs.pop();
        auto img_nodes = ig->GetNodes();

        if(!stlplus::folder_create(dir + "/image_part_" + std::to_string(i))) {
            LOG(WARNING) << "image part " << i << " cannot be created!";
        } 

        for(auto it = img_nodes.begin(); it != img_nodes.end(); ++it) {
            graph::ImageNode inode = it->second;
            string filename = stlplus::filename_part(inode.img_path);
            // cout << "(id, filename): " << inode.id << " " << filename << endl;
            string new_file = dir + "/image_part_" + std::to_string(i) + "/" + filename;
            if(!stlplus::file_copy(inode.img_path, new_file)) {
                LOG(WARNING) << "cannot copy " << inode.img_path << " to " << new_file << endl;
            }
        }
        i++;
    }
}

void ImageCluster::MoveImages(const vector<shared_ptr<ImageGraph>>& image_graphs, const string& dir)
{
    ofstream cluster_list(dir + "/clusters.txt");
    if (!cluster_list.is_open()) {
        LOG(ERROR) << "clusters.txt cannot be created!";
        return;
    }
    ofstream sfmdata_list(dir + "/sfm_datas.txt");
    if (!sfmdata_list.is_open()) {
        LOG(ERROR) << "sfm data list file cannot be created!";
        return;
    }

    for(int i = 0; i < image_graphs.size(); i++) {
        std::vector<graph::ImageNode> img_nodes = image_graphs[i]->SerializeNodes();
        string partial_dir = dir + "/image_part_" + std::to_string(i);
        cluster_list << partial_dir << "\n";
        sfmdata_list << partial_dir + "/reconstruction_sequential/robust.json\n";

        if(!stlplus::folder_create(partial_dir)) {
            LOG(WARNING) << "image part " << i << " cannot be created!";
        }
        image_graphs[i]->ShowInfo(partial_dir + "/graph_cluster.txt");

        if (_cluster_option.copy_images) {
            for(auto inode : img_nodes) {
                string filename = stlplus::filename_part(inode.img_path);
                string new_file = partial_dir + "/" + filename;
                if(!stlplus::file_copy(inode.img_path, new_file)) {
                    LOG(WARNING) << "cannot copy " << inode.img_path << " to " << new_file;
                }
            }
        }
    }
    cluster_list.close();
    sfmdata_list.close();
}

void ImageCluster::MoveImages(const vector<shared_ptr<ImageGraph>>& image_graphs,
                              const string& input_dir, 
                              const string& output_dir)
{
    ofstream cluster_list(input_dir + "/clusters.txt");
    if (!cluster_list.is_open()) {
        LOG(ERROR) << "clusters.txt cannot be created!";
        return;
    }
    ofstream sfmdata_list(input_dir + "/sfm_datas.txt");
    if (!sfmdata_list.is_open()) {
        LOG(ERROR) << "sfm data list file cannot be created!";
        return;
    }

    SfM_Data sfm_data;
    if (!Load(sfm_data, output_dir + "/matches/sfm_data.json", ESfM_Data(VIEWS | INTRINSICS))) {
        LOG(ERROR) << "\nThe input SfM_Data file \"" 
                   << output_dir + "/matches/sfm_data.json" 
                   << "\" cannot be read.";
    }

    for(int i = 0; i < image_graphs.size(); i++) {
        auto img_nodes = image_graphs[i]->GetNodes();
        string partial_dir = input_dir + "/image_part_" + std::to_string(i);
        cluster_list << partial_dir << "\n";
        sfmdata_list << partial_dir + "/reconstruction_sequential/robust.json\n";

        if(!stlplus::folder_create(partial_dir)) {
            LOG(WARNING) << "image part " << i << " cannot be created!";
        }
        image_graphs[i]->ShowInfo(partial_dir + "/graph_cluster.txt");
        GenerateSfMData(sfm_data, img_nodes, partial_dir);

        if (_cluster_option.copy_images) {
            for(auto it = img_nodes.begin(); it != img_nodes.end(); ++it) {
                graph::ImageNode inode = it->second;
                string filename = stlplus::filename_part(inode.img_path);
                // cout << "(id, filename): " << inode.id << " " << filename << endl;
                string new_file = partial_dir + "/" + filename;
                if(!stlplus::file_copy(inode.img_path, new_file)) {
                    LOG(WARNING) << "cannot copy " << inode.img_path << " to " << new_file;
                }
            }
        }
    }

    cluster_list.close();
    sfmdata_list.close();
}

SfM_Data ImageCluster::GenerateSfMData(const SfM_Data& sfm_data,
                     const std::unordered_map<size_t, graph::ImageNode>& img_nodes, 
                     const string& dir)
{
    SfM_Data partial_sfm_data;
    partial_sfm_data.s_root_path = sfm_data.s_root_path;
    Views& views = partial_sfm_data.views;
    Intrinsics& intrinsics = partial_sfm_data.intrinsics;

    // Copy intrinsics
    // for (auto ite = sfm_data.intrinsics.begin(); ite != sfm_data.intrinsics.end(); ++ite) {
    //     intrinsics.insert(make_pair(ite->first, ite->second));
    // }

    // Copy view
    IndexT views_id = views.size();
    for (auto it = img_nodes.begin(); it != img_nodes.end(); ++it) {
        graph::ImageNode img_node = it->second;
        string img1 = stlplus::filename_part(img_node.img_path);
        for (auto ite = sfm_data.views.begin(); ite != sfm_data.views.end(); ++ite) {
            string img2 = stlplus::filename_part(ite->second->s_Img_path);
            if (img1 == img2) {
                views.insert(make_pair(ite->first, ite->second));
                auto intrinsic_it = sfm_data.intrinsics.find(ite->second->id_intrinsic);
                intrinsics.insert(make_pair(intrinsic_it->first, intrinsic_it->second));
                break;
            }
        }
    }

    string path = dir + "/partial_sfm_data.json";
    if (Save(partial_sfm_data, path, ESfM_Data(ALL))) {
        LOG(INFO) << "sfm_data successfully saved in " << path; 
    } else { cout << "Save sfm_data failed!\n"; }
    
    return partial_sfm_data;
}

pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> ImageCluster::BiPartition(const ImageGraph& image_graph)
{
    pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> graph_pair;

    std::unordered_map<int, int> cluster_map = image_graph.NormalizedCut(2);
    queue<shared_ptr<ImageGraph>> graphs = ConstructSubGraphs(image_graph, cluster_map, 2);

    if(graphs.size() != 2) {
        LOG(ERROR) << "Error occurred when bi-partition image graph\n";
        return graph_pair;
    }
    graph_pair.first = graphs.front(); graphs.pop();
    graph_pair.second = graphs.front(); graphs.pop();    
    return graph_pair;
}

bool ImageCluster::HasEdge(const vector<shared_ptr<ImageGraph>>& graphs, 
                           const graph::Edge& edge) const
{
    bool has_edge = false;
    for(int i = 0; i < graphs.size(); i++) {
        auto graph = graphs[i];
        if (graph->HasEdge(edge.src, edge.dst) || 
            graph->HasEdge(edge.dst, edge.src)) return true;
    }
    return has_edge;
}

bool ImageCluster::HasEdge(queue<shared_ptr<ImageGraph>> graphs, const graph::Edge& edge)
{
    std::vector<shared_ptr<ImageGraph>> igs;
    while(!graphs.empty()) {
        igs.push_back(graphs.front());
        graphs.pop();
    }
    bool hasEdge = HasEdge(igs, edge);
    for(auto graph : igs) { graphs.push(graph); }
    return hasEdge;
}

priority_queue<graph::Edge> ImageCluster::DiscardedEdges(const ImageGraph& image_graph, 
                                            const vector<shared_ptr<ImageGraph>>& insize_graphs, 
                                            const queue<shared_ptr<ImageGraph>>& candidate_graphs)
{
    priority_queue<graph::Edge> discarded_edges;
    std::vector<EdgeMap> edge_maps = image_graph.SerializeEdges();

    #pragma omp parallel
    {
        #pragma omp for
        for (int i = 0; i < edge_maps.size(); i++) {
            // auto em = edge_maps[i];
            for (auto em_it = edge_maps[i].begin(); em_it != edge_maps[i].end(); ++em_it) {
                #pragma omp critical
                {
                    graph::Edge edge = em_it->second;
                    if (!HasEdge(insize_graphs, edge) && !HasEdge(candidate_graphs, edge)) {
                    // if (!HasEdge(insize_graphs, edge)) {
                        // as the order of default order of priority_queue is in ascending order,
                        // but the edge with larger number of correspondences should in top of the heap
                        edge.weight = 1.0 / edge.weight;    
                        discarded_edges.push(edge);
                    }
                }
            }
        }
    }
    return discarded_edges;
}

void ImageCluster::CollectDiscardedEdges(const ImageGraph& image_graph,
                                         const queue<shared_ptr<ImageGraph>>& sub_graphs)
{
    auto edges = image_graph.GetEdges();
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        EdgeMap em = it->second;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            Edge edge = em_it->second;
            if (!HasEdge(sub_graphs, edge)) {
                // as the order of default order of priority_queue is in ascending order,
                // but the edge with larger number of correspondences should in top of the heap
                Edge new_edge(edge.src, edge.dst, 1.0 / edge.weight);
                _discarded_edges.push(new_edge);
            }
        }
    }
}

void ImageCluster::CollectDiscardedEdges(const ImageGraph& image_graph,
                                         const ImageGraph& sub_image_graph1,
                                         const ImageGraph& sub_image_graph2)
{
    auto edges = image_graph.GetEdges();
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        EdgeMap em = it->second;
        for (auto em_it = em.begin(); em_it != em.end(); ++em_it) {
            Edge edge = em_it->second;
            if (!sub_image_graph1.HasEdge(edge.src, edge.dst) && 
                !sub_image_graph1.HasEdge(edge.dst, edge.src) &&
                !sub_image_graph2.HasEdge(edge.src, edge.dst) &&
                !sub_image_graph2.HasEdge(edge.dst, edge.src)) {
                // as the order of default order of priority_queue is in ascending order,
                // but the edge with larger number of correspondences should in top of the heap
                Edge new_edge(edge.src, edge.dst, 1.0 / edge.weight);
                _discarded_edges.push(new_edge);
            }
        }
    }
}

int ImageCluster::RepeatedNodeNum(const ImageGraph& image_graphl, const ImageGraph& image_graphr)
{
    int num = 0;
    std::vector<graph::ImageNode> l_nodes = image_graphl.SerializeNodes();
    std::vector<graph::ImageNode> r_nodes = image_graphr.SerializeNodes();
    #pragma omp parallel for reduction(+:num)
    // {
        // #pragma omp for
        for(int i = 0; i < l_nodes.size(); i++) {
            for(int j = 0; j < r_nodes.size(); j++) {
                if(l_nodes[i].id == r_nodes[j].id) num++;
            }
        }
    // }
    return num;
}

pair<graph::ImageNode, shared_ptr<ImageGraph>> ImageCluster::SelectCRGraph(const ImageGraph& image_graph, 
                                                          const vector<shared_ptr<ImageGraph>>& graphs, 
                                                          const graph::Edge& edge)
{
    graph::ImageNode unselected_node, selected_node;
    bool find = false;
    srand((unsigned)time(NULL));
    int ran = rand() % 2;

    for(int i = 0; i < graphs.size(); i++) {
        unselected_node = (ran == 0) ? image_graph.GetNode(edge.dst) : image_graph.GetNode(edge.src);
        selected_node = (ran == 0) ? graphs[i]->GetNode(edge.src) : graphs[i]->GetNode(edge.dst);
        int repeatedNodeNum = 0;
        if(selected_node.id != -1) {
            #pragma omp parallel for reduction(+:repeatedNodeNum)
            // {
                // #pragma omp for
                for(int j = 0; j < graphs.size(); j++) {
                    if(i != j) {
                        repeatedNodeNum += RepeatedNodeNum(*graphs[i], *graphs[j]);
                        // if((float)repeatedNodeNum / (float)graphs[i].GetNodeSize() > 
                        // _cluster_option.completeness_ratio) {
                        //     break;
                        // }
                    }
                }
            // }
            if((float)repeatedNodeNum / (float)graphs[i]->GetNodesNum() < 
                _cluster_option.completeness_ratio) {
                return make_pair(unselected_node, graphs[i]);
            }
        }
    }
    shared_ptr<ImageGraph> rep(new ImageGraph());
    return make_pair(graph::ImageNode(), rep);
}

bool ImageCluster::IsSatisfyCompleteConstraint(const ImageGraph& image_graph, 
                                               const vector<shared_ptr<ImageGraph>>& graphs)
{
    int repeatedNodeNum = 0;
    for(int i = 0; i < graphs.size(); i++) {
        #pragma omp parallel for reduction(+:repeatedNodeNum)
        // {
            // #pragma omp for
            for(int j = 0; j < graphs.size(); j++) {
                if(i != j) {
                    repeatedNodeNum += RepeatedNodeNum(*graphs[i], *graphs[j]);
                    // if((float)repeatedNodeNum / (float)graphs[i].GetNodeSize() > 
                    //    _cluster_option.completeness_ratio) {
                    //     break;
                    // }
                }
            }
        // }
        if((float)repeatedNodeNum / (float)graphs[i]->GetNodesNum() < 
            _cluster_option.completeness_ratio) {
            return true;
        }
    }
    return false;
}

vector<shared_ptr<ImageGraph>> ImageCluster::ExpanImageCluster(const ImageGraph& image_graph, 
                                                     queue<shared_ptr<ImageGraph>> image_graphs)
{
    vector<shared_ptr<ImageGraph>> insize_graphs;
    queue<shared_ptr<ImageGraph>>& candidate_graphs = image_graphs;

    while(!candidate_graphs.empty()) {
        while(!candidate_graphs.empty()) {
            shared_ptr<ImageGraph> ig = candidate_graphs.front();
            candidate_graphs.pop();
            if(ig->GetNodesNum() <= _cluster_option.cluster_upper_size) {
                insize_graphs.push_back(ig);
            } else {
                pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> ig_pair = BiPartition(*ig);
                candidate_graphs.push(ig_pair.first);
                candidate_graphs.push(ig_pair.second);
                CollectDiscardedEdges(*ig, *(ig_pair.first), *(ig_pair.second));
            }
        }

        // Graph expansion
        // priority_queue<graph::Edge> discarded_edges = 
        //     DiscardedEdges(image_graph, insize_graphs, candidate_graphs);
        
        while(!_discarded_edges.empty()) {
            graph::Edge discarded_edge = _discarded_edges.top();
            _discarded_edges.pop();
            #ifdef OUTPUT_DISCARDED_EDGES
            LOG(INFO) << "Discarded Edges: " << discarded_edge.src << ", " 
                      << discarded_edge.dst << ": " << discarded_edge.weight; 
            #endif
            pair<graph::ImageNode, shared_ptr<ImageGraph>> crp_graph = 
                SelectCRGraph(image_graph, insize_graphs, discarded_edge);
            graph::ImageNode unselected_node = crp_graph.first;
            shared_ptr<ImageGraph> selected_graph = crp_graph.second;
            if(unselected_node.id != -1) {
                if(selected_graph->GetNode(unselected_node.id).id == -1) {
                    selected_graph->AddNode(unselected_node);
                }
                graph::Edge edge(discarded_edge.src, discarded_edge.dst, 1.0 / discarded_edge.weight);
                selected_graph->AddEdge(edge);
            }
        }
        // After graph expansion, there may be some image graphs that don't
        // satisfy the size constraint, check this condition
        std::vector<shared_ptr<ImageGraph>>::iterator igIte;
        for(igIte = insize_graphs.begin(); igIte != insize_graphs.end();) {
            // Make a little relax of original constraint condition
            if((*igIte)->GetNodesNum() > _cluster_option.cluster_upper_size 
                + (int)(_cluster_option.cluster_upper_size * _cluster_option.relax_ratio)) {
            // if((*igIte)->GetNodeSize() > _cluster_option.cluster_upper_size) {
                candidate_graphs.push(*igIte);
                igIte = insize_graphs.erase(igIte);
            }
            else igIte++;
        }
    }
    return insize_graphs;
}

void ImageCluster::NaiveImageCluster(queue<shared_ptr<ImageGraph>> image_graphs, 
                                                     const string& dir, 
                                                     const size_t& cluster_num)
{
    MoveImages(image_graphs, dir);
}

queue<shared_ptr<ImageGraph>> ImageCluster::ConstructSubGraphs(ImageGraph image_graph, 
                                                     const std::unordered_map<int, int>& clusters, 
                                                     const size_t& cluster_num)
{
    queue<shared_ptr<ImageGraph>> image_graphs;
    auto nodes = image_graph.GetNodes();
    auto edges = image_graph.GetEdges();
    
    #pragma omp parallel
    {
        #pragma omp for
        for(int i = 0; i < cluster_num; i++) {
            shared_ptr<ImageGraph> ig(new ImageGraph());
            // Add nodes
            for (auto it = clusters.begin(); it != clusters.end(); ++it) {
                if (it->second == i) { ig->AddNode(nodes[it->first]); }
            }

            // Add edges
            std::vector<graph::ImageNode> ig_nodes = ig->SerializeNodes();
            for(int l = 0; l < ig_nodes.size(); l++) {
                for(int r = l + 1; r < ig_nodes.size(); r++) {
                    auto it = edges[ig_nodes[l].id].find(ig_nodes[r].id);
                    if(it != edges[ig_nodes[l].id].end()) {
                        graph::Edge edge(ig_nodes[l].id, ig_nodes[r].id, it->second.weight);
                        ig->AddEdge(edge);
                    }
                }
            }
            image_graphs.push(ig);
        }
    }
    return image_graphs;
}

}   // namespace geometry
}   // namespace GraphSfM