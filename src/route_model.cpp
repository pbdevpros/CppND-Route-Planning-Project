#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // create a new RouteModel::Node
    int i = 0;
    for (auto mNode : this->Nodes()) {
        RouteModel::Node rm_Node( i, this, mNode);
        m_Nodes.push_back(rm_Node);
        i++;
    }
    CreateNodeToRoadHashmap();
}

float RouteModel::Node::distance( RouteModel::Node node) const {
    float dist = std::sqrt(std::pow(x-node.x, 2) + std::pow(y-node.y, 2));
    return dist;
}


void RouteModel::CreateNodeToRoadHashmap() {
    // const std::vector<Road> roads = Roads();
    for ( const Model::Road &road : Roads() ) {
        if ( road.type != Model::Road::Type::Footway ) {
            for ( int node_idx : Ways()[road.way].nodes )  {
                if ( node_to_road.find(node_idx) == node_to_road.end() ) {
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    RouteModel::Node *closest_node = nullptr;
    Node node;
    for (auto node_x : node_indices) {
        // find the closest unvisted node...
        node = parent_model->SNodes()[node_x] ;
        if( !node.visited && ( this->distance(node) != 0 ) ) {
            if ( closest_node == nullptr || ( this->distance(node) < this->distance(*closest_node) ) ) {
                closest_node = &parent_model->SNodes()[node_x];
            }
        }   
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors() {
    for ( auto & road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if ( new_neighbor ) {
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    RouteModel::Node tempNode;
    tempNode.x = x;
    tempNode.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads() ) {
        if ( road.type != Model::Road::Type::Footway ) {
            for ( auto node_index : Ways()[road.way].nodes ) {
                dist = tempNode.distance(SNodes()[node_index]);
                if ( dist < min_dist ) {
                    closest_idx = node_index;
                    min_dist = dist;
                }
            }
        }
    }
    return SNodes()[closest_idx];
}