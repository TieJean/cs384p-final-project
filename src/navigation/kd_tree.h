
   
#include <iostream>
#include <vector>
#include <math.h>

#ifndef KD_TREE_H
#define KD_TREE_H
#define KDTREE_TEM template <class NodeClass, typename ElemType, size_t N>
#define KDTREE_FUN KDTree<NodeClass, ElemType, N>

namespace RRTPlanner {

  KDTREE_TEM
  class KDTree{
  
  private:
    struct Node{
      NodeClass* node;
      Node* l_child, *r_child;
    };

    Node* root;

    void insert(Node* &, NodeClass*, int count_dim=0);
    void traverse(Node* , int coutn_dim=0);
    NodeClass* findMin(Node*, int, int count_dim=0);
    void NNSearch(Node*, NodeClass*, NodeClass* &, double&, int count_dim=0);
    void deleteTree(Node*);
  public:  
    KDTree(){
      root = nullptr;
    };
    ~KDTree(){

    };

    void deleteTree(){
      this->deleteTree(this->root);
      this->root = nullptr;
    };
    
    void insert(NodeClass* x_new){
      this->insert(this->root, x_new, 0);
    };

    void traverse(){
      this->traverse(this->root, 0);
    };
    NodeClass* findMin(int dim){
      return this->findMin(this->root, dim, 0);
    };
    void NNSearch(NodeClass* query_code, NodeClass* & nearest_neighbor, double& min_dist){
      this->NNSearch(this->root, query_code, nearest_neighbor, min_dist, 0);
    };

    // Compare the node based on counting dimension
    NodeClass* minNode(int dim, NodeClass* a_node, NodeClass* b_node){
      if (a_node == nullptr and b_node ==nullptr){
	return nullptr;
      }
      if (a_node == nullptr){
	return b_node;
      }
      if (b_node == nullptr){
	return a_node;
      }
    
      if (a_node->x[dim] < b_node->x[dim]){
	return a_node;
      }
      else{
	return b_node;
      }
    };
    // reference to pointer : [https://docs.microsoft.com/en-us/cpp/cpp/references-to-pointers?view=vs-2019]
  };

  KDTREE_TEM
  void KDTREE_FUN::deleteTree(Node* node){
    if (node != NULL) {
      /* first delete both subtrees */
      deleteTree(node->l_child);
      node->l_child = nullptr;
      deleteTree(node->r_child);
      node->r_child = nullptr;
      delete(node);
    }
  };
  
  KDTREE_TEM
  void KDTREE_FUN::insert(KDTREE_FUN::Node* & r, NodeClass* x_new, int count_dim){
    if (r == nullptr){
      r = new Node();
      r->node = x_new;
      r->l_child = nullptr;
      r->r_child = nullptr;
      return;
    }
    if (r->node->x[count_dim] < x_new->x[count_dim]){
      insert(r->r_child, x_new, (count_dim+1) % N);
    }
    else{
      insert(r->l_child, x_new, (count_dim+1) % N);
    }
  };

  KDTREE_TEM
  void KDTREE_FUN::traverse(Node* r, int count_dim){
    if (r->l_child != nullptr){
      traverse(r->l_child, (count_dim+1)%N);
    };
    
    std::cout << r->node << ", Count_Dimension: "<< count_dim << std::endl;
    if (r->r_child != nullptr){
      traverse(r->r_child, (count_dim+1)%N);
    };
  };

  KDTREE_TEM
  NodeClass* KDTREE_FUN::findMin(Node *r, int dim, int count_dim)
  {
    if (r==nullptr) return nullptr;
    if (dim == count_dim){
      if (r->l_child == nullptr){
	return r->node;
      }
      else{
	return findMin(r->l_child, dim, (count_dim+1) % N);
      }
      
    }
    else{
      NodeClass* left_min, *right_min;
      if (r->l_child!=nullptr){
	left_min = minNode(dim, findMin(r->l_child, dim, (count_dim+1) %N), r->node);
      }
      else{
	left_min = r->node;
      }
      if (r->r_child!=nullptr){
	right_min = minNode(dim, findMin(r->r_child, dim, (count_dim+1) %N), r->node);
      }
      else{
	right_min = r->node;
      }
      return minNode(dim, left_min, right_min);
    }
    
  };

  KDTREE_TEM
  void KDTREE_FUN::NNSearch(Node* r,NodeClass* query_node, NodeClass* & nearest_neighbor, double & min_dist,  int count_dim){
    if (r==nullptr){
      return;
    }
    double dist = (*r->node).dist(*query_node);
    if (dist < min_dist){
      min_dist = dist;
      nearest_neighbor = r->node;
    }
    bool search_left_first = false;
    if (query_node->x[count_dim] < r->node->x[count_dim]){
      search_left_first = true;
    }
    if (search_left_first){
      if (query_node->x[count_dim] - min_dist < r->node->x[count_dim] && r->l_child!=nullptr){
	NNSearch(r->l_child, query_node, nearest_neighbor, min_dist, (count_dim + 1)%N);
      }
      if (query_node->x[count_dim] + min_dist > r->node->x[count_dim] && r->r_child!=nullptr){
	NNSearch(r->r_child, query_node, nearest_neighbor, min_dist, (count_dim + 1)%N);
      }
	
    }
    else {
      if (query_node->x[count_dim] + min_dist > r->node->x[count_dim] && r->r_child!=nullptr){
	NNSearch(r->r_child, query_node, nearest_neighbor, min_dist, (count_dim + 1)%N);
      }	
      if (query_node->x[count_dim] - min_dist < r->node->x[count_dim] && r->l_child!=nullptr){
	NNSearch(r->l_child, query_node, nearest_neighbor, min_dist, (count_dim + 1)%N);
      }
    }
  };
}

#endif