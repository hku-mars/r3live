/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *   
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/
/****************************************************************************
  History

$Log: not supported by cvs2svn $

****************************************************************************/

#ifndef VCG_MT_H
#define VCG_MT_H

#include <vector>
#include <queue> 
#include <map>

#include <vcg/space/sphere3.h>
#include <vcg/space/point3.h>
#include <wrap/gui/frustum.h>

namespace vcg {  

  /** \addtogroup mt */
/*@{*/
    /**
        The templated class for representing a mt structure.
        The class is templated over the Cell class which MUST implement this functions:
        float Error(); 
        float Distance(Cell &c, Point3f &p);
        see mt_file.h for an example.        
     */

template <class C> class MT {
public:
  typedef C Cell;

private:

  class Frag:public std::vector<Cell *> {};

  struct Node {  
    std::vector<Node *> in;
    std::vector<Node *> out;
    std::vector<Frag> frags;    
    float error;
    bool visited;        
  };

  std::vector<Node> nodes;

public:

  /** Update class for defining the history.
    Notice how the Update class holds pointers. 
     You are responsible of holding the Cell structure and rebuild the structure
     if the pointers change.
     */
  class Update {
  public:  
    std::vector<Cell *> erased; 
    std::vector<Cell *> created;    
  };
  
  ///build mt strucure from a sequence of updates.
  void Load(std::vector<Update> &updates) {    
    //The last update erases everything.
    assert(updates[0].erased.size() == 0);

    //maps cell -> node containing it
    std::map<Cell *, unsigned int> cell_node;   
    nodes.resize(updates.size());

    //building fragments and nodes.
    unsigned int current_node = 0;
    std::vector<Update>::iterator u;
    for(u = updates.begin(); u != updates.end(); u++) {      
    
      Node &node = nodes[current_node];
      node.error = 0;
        
      //created cells belong to this node, we look also for max error.      
      for(unsigned int i = 0; i < (*u).created.size(); i++) {
        Cell *cell = (*u).created[i];
        if(cell->Error() > node.error)
          node.error = cell->Error();
          
        cell_node[cell] = current_node;        
      }
        
      //Every erased cell already belonged to a node.
      //we record for each node its cells.
      std::map<unsigned int, std::vector<Cell *> > node_erased;      
        
      for(unsigned int i = 0; i < (*u).erased.size(); i++) {
        Cell *cell = (*u).erased[i];
        assert(cell_node.count(cell));
        node_erased[cell_node[cell]].push_back(cell);               
      }      

      //for every node with erased cells we build a frag and put the corresponding cells in it.      
      std::map<unsigned int, std::vector<Cell *> >::iterator e;
      for(e = node_erased.begin(); e != node_erased.end(); e++) {
        //Build a new Frag.
        Frag fr;
        float max_err = -1;

        //Fill it with erased cells.
        std::vector<Cell *> &cells = (*e).second;
        std::vector<Cell *>::iterator k;
        for(k = cells.begin(); k != cells.end(); k++) {
          Cell *cell = (*k);
          fr.push_back(cell);
          if(cell->Error() > max_err)
            max_err = cell->Error();            
        }
        
        //Add the new Frag to the node.
        unsigned int floor_node = (*e).first;
        Node &oldnode = nodes[floor_node];
        oldnode.frags.push_back(fr);
        if(node.error < max_err)
          node.error = max_err;
      
        //Update in and out of the nodes.
        node.in.push_back(&oldnode);
        oldnode.out.push_back(&node);
      }
      current_node++;
    }
  }
  void Clear() {
    nodes.clear();
  }
    
  ///error based extraction 
  template <class CONT> void Extract(CONT &selected, float error) {
    std::vector<Node>::iterator n;
    for(n = nodes.begin(); n != nodes.end(); n++)
      (*n).visited = false;

    std::queue<Node *> qnodo;
	  qnodo.push(&nodes[0]);
    nodes[0].visited = true;

	  for( ; !qnodo.empty(); qnodo.pop()) {
      Node &node = *qnodo.front();
                                                                    
      std::vector<Frag>::iterator fragment;
      std::vector<Node *>::iterator on;
		  for(on = node.out.begin(), fragment = node.frags.begin(); on != node.out.end(); ++on, ++fragment) {
        if((*on)->visited) continue;
			        
        if(error < (*on)->error) { //need to expand this node.
				  qnodo.push(*on);
          (*on)->visited = 1;
			  } else {
          vector<Cell *>::iterator cell;
          for(cell=(*fragment).begin(); cell != (*fragment).end(); ++cell) 
            selected.push_back(*cell);                   
			  }
		  }
    }  
  }    

  //custom extraction
  template <class CONT, class POLICY> void Extract(CONT &selected, POLICY &policy) {
    std::vector<Node>::iterator n;
    for(n = nodes.begin(); n != nodes.end(); n++)
      (*n).visited = false;

    std::queue<Node *> qnodo;
	  qnodo.push(&nodes[0]);
    nodes[0].visited = true;

    for( ; !qnodo.empty(); qnodo.pop()) {
      Node &node = *qnodo.front();   

      std::vector<Frag>::iterator i;
      std::vector<Node *>::iterator on;
      for(i = node.frags.begin(), on = node.out.begin(); i != node.frags.end(); i++, on++) {
        if((*on)->visited) continue;
        Frag &frag = (*i);
        std::vector<Cell *>::iterator cell;
        for(cell = frag.begin(); cell != frag.end(); cell++) {              
          if(policy.Expand(*cell))          
            visit(*on, qnodo);          
        }
      }
    }
    Extract(selected);
  }

  //extract using a user class which must provides:
    //bool refineCell(Cell *cell, Frustum &frustum);
    //void addCell(Cell *cell);
    //void removeCell(Cell *cell);
  //template <class CONT> void extract(std::vector<Cell *> &selected, Frustum &frustum, T &error);      

  ///extract using the visited flag in nodes to a STL container of Cell *
  template <class CONT> void Extract(CONT &selected) {
    selected.clear();
    std::vector<Node>::iterator i;
    for(i = nodes.begin(); i != nodes.end(); i++) {
      Node &node = *i;
      if(!node.visited)       
        continue;                  
    
      std::vector<Node *>::iterator n;
      std::vector<Frag>::iterator f;        
      for(n = node.out.begin(), f = node.frags.begin(); n != node.out.end(); n++, f++) {
        if(!(*n)->visited || (*n)->error == 0) {
          vector<Cell *>::iterator c;
          Frag &frag = (*f);
          for(c = frag.begin(); c != frag.end(); c++)            
             selected.insert(selected.end(), *c);        
        }      
      } 
    }
  }

  ///add a node to the queue recursively mantaining coherence.
  void Visit(Node *node, std::queue<Node *> &qnode) {    
    std::vector<Node *>::iterator n;
    for(n = node->in.begin(); n != node->in.end(); n++) 
      if(!(*n)->visited) 
        visit(*n, qnode);  

    node->visited = true;
    qnode.push(node);
  }

    
};

template <class C> class FrustumPolicy {
public:
  typedef C Cell;

  Frustumf frustum;
  ///target error in screen space (pixels)
  float error;

  bool Expand(Cell *cell) {
    if(cell->Error() == 0) return false;
    float dist = Distance(*cell, frustum.ViewPoint());
    /*Point3f line = frustum.viewPoint() - cell->sphere.center;
    float dist = line.Norm() - cell->sphere.radius;  */
    if(dist < 0) return true;
    dist = pow(dist, 1.2);
    float res =  cell->Error() / (frustum.Resolution()*dist);
    return res;
  }
};

///really simple example policy
template <class C> class DiracPolicy {
public:
  typedef C Cell;
  
  Point3f focus;
  ///max error in object space at distance 0
  float error;

  DiracPolicy(Point3f &f = Point3f(0, 0, 0), float e = 0): focus(f), error(e) {}

  bool Expand(Cell *cell) {
    if(cell->Error() == 0) return false;
    float dist = Distance(*cell, focus);
    if(dist > 0) return true;
    return false;
  }
};

}//namespace vcg

#endif