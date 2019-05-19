#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>

#include "graph/viewing_graph.h"
#include "graph/graph.h"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace openMVG::sfm;
using namespace GraphSfM;
using namespace GraphSfM::graph;
using namespace std;

#define USING_PAIGE

// command line options defined with gflags
DEFINE_string(matches_dir, "", "folder that stores the matches file");
DEFINE_string(sfm_data_file, "", "path that stores the sfm_data.json generated with OpenMVG format");
DEFINE_string(similarity_match_file, "", "path that stores the similarity score");
DEFINE_uint32(community_number, 5, "the number that we want to divide the graph into");
DEFINE_bool(loop_consistency_enable, true, "whether to check loop consistency");
DEFINE_bool(rank_edges_enable, false, "Rank edges by similarity score");
DEFINE_bool(retain_singleton_nodes_enable, false, "retain singleton nodes after online mst");
DEFINE_bool(strong_triplet_expansion_enable, true, "whether to use stong triplet expansion");
DEFINE_bool(graph_reinforcement_enable, true, "whether to use graph reinforcement");

// sort in descending order by edge weight
bool EdgeCompare(const Edge& edge1, const Edge& edge2)
{
    return edge1.weight > edge2.weight;
}

vector<vector<float>> RankedEdges(const Graph<ImageNode, Edge>& graph)
{
    vector<vector<Edge>> sorted_edges;
    sorted_edges.resize(graph.GetNodes().size());

    vector<unordered_map<size_t, Edge>> edges = graph.SerializeEdges();
    for (int i = 0; i < edges.size(); i++) {
        vector<Edge> edge_list;
        for (auto it = edges[i].begin(); it != edges[i].end(); it++) {
            edge_list.push_back(it->second);
        }
        sort(edge_list.begin(), edge_list.end(), EdgeCompare);
        sorted_edges[i] = edge_list;
    }

    vector<vector<float>> rank_map;
    rank_map.resize(graph.GetNodes().size());
    for (size_t i = 0; i < sorted_edges.size(); i++) {
        vector<float> ranks(graph.GetNodes().size());
        for (size_t k = 0; k < sorted_edges[i].size(); k++) {
            Edge edge = sorted_edges[i][k];
            size_t j = edge.dst;
            ranks[j] = (float)k;
        }
        rank_map[i] = ranks;
    }
    return rank_map;
}

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    string matches_dir = FLAGS_matches_dir;
    string sfmdata_filename = FLAGS_sfm_data_file;
    string similarity_match_file = FLAGS_similarity_match_file;
    int community_number = FLAGS_community_number;   // a parameter we need to set by experience
    bool rank_edges_enable = FLAGS_rank_edges_enable;
    bool retain_singleton_nodes_enable = FLAGS_retain_singleton_nodes_enable;
    bool strong_triplet_expansion_enable = FLAGS_strong_triplet_expansion_enable;
    bool graph_reinforcement_enable = FLAGS_graph_reinforcement_enable;
    // if (argc > 4) community_number = atoi(argv[4]);

    FLAGS_log_dir = matches_dir;
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    if (stlplus::file_exists(matches_dir + "/matches.f.txt")) {
        LOG(INFO) << "original matches file exists\n";
        return 1;
    }

    ifstream in(similarity_match_file);
    if (!in.is_open()) {
        LOG(ERROR) << similarity_match_file << " can't be opened!\n";
        return 0;
    }
    
    SfM_Data sfm_data;
    if (!Load(sfm_data, sfmdata_filename, sfm::ESfM_Data(ESfM_Data::VIEWS))) {
        LOG(ERROR) << "\nThe Input SfM_Data file \"" << sfmdata_filename 
                   << "\" Cannot be read.\n";
        return false;
    }

    Views views = sfm_data.GetViews();
    string root_path = sfm_data.s_root_path;

    // graph initialization
    Graph<ImageNode, Edge> graph;
    size_t i, j;
    float score;    // similarity score

    for (auto it = views.begin(); it != views.end(); it++) {
        string abs_path = root_path + "/" + it->second->s_Img_path;
        size_t view_id = it->second->id_view;
        ImageNode node(view_id, abs_path);
        graph.AddNode(node);
    }

    while (in >> i >> j >> score) {
        if (i >= j) continue;
        Edge edge(i, j, 1.0 / score);
        graph.AddEdge(edge);
    }
    // graph.ShowInfo();

    // modify edge weight by the rank of similarity score
    if (rank_edges_enable) {
        vector<vector<float>> ranked_edges = RankedEdges(graph);
        for (int i = 0; i < ranked_edges.size(); i++) {
            for (int j = i + 1; j < ranked_edges[i].size(); j++) {
                float rank_ij = ranked_edges[i][j];
                float rank_ji = ranked_edges[j][i];
                float weight = sqrt((rank_ij * rank_ij + rank_ji * rank_ji) / 2.0);
                Edge edge(i, j, weight);
                graph.AlterEdge(edge);
            }
        }
        // graph.ShowInfo();
    }

    // Construct viewing graph
    ViewingGraph viewing_graph(matches_dir, sfmdata_filename);
    viewing_graph.SetInitialGraph(graph);

    ViewingGraphOption vg_option;
    vg_option.component_number = community_number;
    vg_option.loop_consistency_enable = FLAGS_loop_consistency_enable;
    viewing_graph.SetVGOption(vg_option);

    // online minimum spanning tree
    viewing_graph.OnlineMST();
    viewing_graph.ExportToDisk(matches_dir, "online_mst_graph");

    if (retain_singleton_nodes_enable) {
        viewing_graph.RetainSingletonNodes();
        viewing_graph.ExportToDisk(matches_dir, "retain_singleton_nodes_graph");
    }

    if (strong_triplet_expansion_enable) {
        viewing_graph.StrongTripletsExpansion();
        viewing_graph.ExportToDisk(matches_dir, "triplet_expansion_graph");
    }

    if (graph_reinforcement_enable) {
        // while (viewing_graph.GetComponentNumber() > 1) {
            // viewing_graph.GraphReinforcement();
        // }
        viewing_graph.GraphReinforcement();
        LOG(INFO) << "component number: " << viewing_graph.GetComponentNumber();
        viewing_graph.ExportToDisk(matches_dir, "graph_reinforcement_graph");
    }
    viewing_graph.ShowStatistics();
}
