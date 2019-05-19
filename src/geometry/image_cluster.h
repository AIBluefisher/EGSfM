#ifndef GEOMETRY_GRAPH_CLUSTER_H
#define GEOMETRY_GRAPH_CLUSTER_H

#include <string>
#include <fstream>
#include <cstring>
#include <vector>
#include <memory>
#include <queue>
#include <utility>

#include "graph/graph.h"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"

using namespace Eigen;

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace openMVG::matching;

using namespace std;
using namespace GraphSfM::graph;

typedef Graph<ImageNode, Edge> ImageGraph;
typedef std::unordered_map<size_t, Edge> EdgeMap;

namespace GraphSfM {
namespace geometry {

struct ImageClusterOption
{
    size_t cluster_upper_size;
    float completeness_ratio;
    float relax_ratio;
    bool copy_images;

    ImageClusterOption(const size_t& upper_size = 100, const float& cr = 0.7, 
                       const float& rr = 0.35, const bool& ci = false)
                       : cluster_upper_size(upper_size), 
                         completeness_ratio(cr),
                         relax_ratio(rr),
                         copy_images(ci) { }

    ImageClusterOption(const ImageClusterOption& option)
    {
        cluster_upper_size = option.cluster_upper_size;
        completeness_ratio = option.completeness_ratio;
        relax_ratio = option.relax_ratio;
        copy_images = option.copy_images;
    }
};

class ImageCluster 
{
private:
    ImageClusterOption _cluster_option;
    priority_queue<graph::Edge> _discarded_edges;

public:

    ImageCluster(size_t upper = 100, float cr = 0.7);

    ImageCluster(const ImageClusterOption& cluster_option);

    void SetClusterOption(const ImageClusterOption& cluster_option);

    ImageGraph BuildGraph(const string& image_list, const string& voc_file) const;

    void MoveImages(queue<shared_ptr<ImageGraph>>& image_graphs, const string& dir);  

    void MoveImages(const vector<shared_ptr<ImageGraph>>& image_graphs, const string& dir);  

    void MoveImages(const vector<shared_ptr<ImageGraph>>& image_graphs, 
                    const string& input_dir, 
                    const string& output_dir); 

    SfM_Data GenerateSfMData(const SfM_Data& sfm_data,
                         const std::unordered_map<size_t, graph::ImageNode>& img_nodes, 
                         const string& dir);


    pair<shared_ptr<ImageGraph>, shared_ptr<ImageGraph>> BiPartition(const ImageGraph& image_graph); 

    bool HasEdge(const vector<shared_ptr<ImageGraph>>& graphs,
                 const graph::Edge& edge) const;  

    bool HasEdge(queue<shared_ptr<ImageGraph>> graphs, const graph::Edge& edge);

    /**
     * @brief Collect discarded edges in graph division
     * @param imageGraph:
     * @param insizeGraphs:
     * @param candidateGraphs:
     * @return
     */
    priority_queue<graph::Edge> DiscardedEdges(const ImageGraph& image_graph, 
                                               const vector<shared_ptr<ImageGraph>>& insize_graphs, 
                                               const queue<shared_ptr<ImageGraph>>& candidate_graphs);
                                               
    void CollectDiscardedEdges(const ImageGraph& image_graph,
                               const queue<shared_ptr<ImageGraph>>& sub_graphs);
    void CollectDiscardedEdges(const ImageGraph& image_graph,
                               const ImageGraph& sub_image_graph1,
                               const ImageGraph& sub_image_graph2);

    bool IsSatisfyCompleteConstraint(const ImageGraph& image_graph, 
                                     const vector<shared_ptr<ImageGraph>>& graphs);
    /**
     * @brief Select a image graph randomly which satisfy the completeness ratio
     * @param graphs:
     * @param edge:
     * @return
     */
    pair<graph::ImageNode, shared_ptr<ImageGraph>> SelectCRGraph(const ImageGraph& image_graph, 
                                                            const vector<shared_ptr<ImageGraph>>& graphs, 
                                                            const graph::Edge& edge); 

    int RepeatedNodeNum(const ImageGraph& image_graphl, const ImageGraph& image_graphr); 


    vector<shared_ptr<ImageGraph>> ExpanImageCluster(const ImageGraph& image_graph, 
                                                     queue<shared_ptr<ImageGraph>> image_graphs); 

    void NaiveImageCluster(queue<shared_ptr<ImageGraph>> image_graphs, 
                                                     const string& dir, 
                                                     const size_t& cluster_num); 

    queue<shared_ptr<ImageGraph>> ConstructSubGraphs(ImageGraph image_graph, 
                                                     const std::unordered_map<int, int>& clusters, 
                                                     const size_t& cluster_num); 
    };

}   // namespace geometry
}   // namespace GraphSfM

#endif