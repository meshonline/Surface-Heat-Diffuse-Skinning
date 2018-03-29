//
//  Octree.h
//
//  Created by Eduardo Poyart on 6/4/12.
//

/*
 Copyright (c) 2012, Eduardo Poyart.
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are
 met:
 
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the following disclaimer
 in the documentation and/or other materials provided with the
 distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef Octree_h
#define Octree_h

#include <assert.h>
#include <stack>

#define COMPUTE_SIDE(i, bit, p, mid, newMin, newMax) \
if (p >= mid)         \
{                     \
i |= bit;         \
newMin = mid;     \
}                     \
else                  \
{                     \
newMax = mid;     \
}


template <class N>
class Octree
{
protected:
  struct Point
  {
    float x;
    float y;
    float z;
    Point(const Point& p2): x(p2.x), y(p2.y), z(p2.z) {}
    Point& operator=(const Point& p2) { x = p2.x; y = p2.y; z = p2.z; return *this;}
    Point(float in_x, float in_y, float in_z): x(in_x), y(in_y), z(in_z) {}
    Point(const float p2[3]): x(p2[0]), y(p2[1]), z(p2[2]) {}
    operator float*() { return &x; }
    operator const float*() const { return &x; }
    Point operator+(const Point& p2) const { return Point(x+p2.x, y+p2.y, z+p2.z); }
    Point operator-(const Point& p2) const { return Point(x-p2.x, y-p2.y, z-p2.z); }
    Point operator*(float f) const { return Point(x*f, y*f, z*f); }
    bool operator< (const Point& p2) const { return x <  p2.x && y <  p2.y && z <  p2.z; }
    bool operator>=(const Point& p2) const { return x >= p2.x && y >= p2.y && z >= p2.z; }
  };
  
  struct OctreeNode
  {
    N _nodeData;
    OctreeNode* _children[8];
    OctreeNode()
    {
      for (int i = 0; i < 8; i++)
        _children[i] = 0;
    }
    virtual ~OctreeNode()
    {
      for (int i = 0; i < 8; i++)
        if (_children[i])
          delete _children[i];
    }
  };
  
  Point _min;
  Point _max;
  Point _cellSize;
  OctreeNode* _root;
  
public:
  Octree(float min[3], float max[3], float cellSize[3]): _min(min), _max(max), _cellSize(cellSize), _root(0) {}
  virtual ~Octree() { delete _root; }
  
  class Callback
  {
  public:
    // Return value: true = continue; false = abort.
    virtual bool operator()(const float min[3], const float max[3], N& nodeData) = 0;
  };
  
  N& getCell(const float pos[3], Callback* callback = NULL)
  {
    Point ppos(pos);
    assert(ppos >= _min && ppos < _max);
    Point currMin(_min);
    Point currMax(_max);
    Point delta = _max - _min;
    if (!_root)
      _root = new OctreeNode();
    OctreeNode* currNode = _root;
    while (delta >= _cellSize)
    {
      bool shouldContinue = true;
      if (callback)
        shouldContinue = callback->operator()(currMin, currMax, currNode->_nodeData);
      if (!shouldContinue)
        break;
      Point mid = (delta * 0.5f) + currMin;
      Point newMin(currMin);
      Point newMax(currMax);
      int index = 0;
      COMPUTE_SIDE(index, 1, ppos.x, mid.x, newMin.x, newMax.x)
      COMPUTE_SIDE(index, 2, ppos.y, mid.y, newMin.y, newMax.y)
      COMPUTE_SIDE(index, 4, ppos.z, mid.z, newMin.z, newMax.z)
      if (!(currNode->_children[index]))
        currNode->_children[index] = new OctreeNode();
      currNode = currNode->_children[index];
      currMin = newMin;
      currMax = newMax;
      delta = currMax - currMin;
    }
    return currNode->_nodeData;
  }
  
  void traverse(Callback* callback, const bool recursive = false)
  {
    assert(callback);
    // non-recursive method is safer with deep recursive tasks.
    if (recursive) {
      traverseRecursive(callback, _min, _max, _root);
    } else {
      traverseNoRecursive(callback, _min, _max, _root);
    }
  }
  
  void clear()
  {
    delete _root;
    _root = NULL;
  }
  
  class Iterator
  {
  public:
    Iterator getChild(int i)
    {
      return Iterator(_currNode->_children[i]);
    }
    N* getData()
    {
      if (_currNode)
        return &_currNode->_nodeData;
      else return NULL;
    }
  protected:
    OctreeNode* _currNode;
    Iterator(OctreeNode* node): _currNode(node) {}
    friend class Octree;
  };
  
  Iterator getIterator()
  {
    return Iterator(_root);
  }
  
protected:
  void traverseRecursive(Callback* callback, const Point& currMin, const Point& currMax, OctreeNode* currNode)
  {
    if (!currNode)
      return;
    bool shouldContinue = callback->operator()(currMin, currMax, currNode->_nodeData);
    if (!shouldContinue)
      return;
    Point delta = currMax - currMin;
    Point mid = (delta * 0.5f) + currMin;
    traverseRecursive(callback, currMin, mid, currNode->_children[0]);
    traverseRecursive(callback, Point(mid.x, currMin.y, currMin.z),
                      Point(currMax.x, mid.y, mid.z), currNode->_children[1]);
    traverseRecursive(callback, Point(currMin.x, mid.y, currMin.z),
                      Point(mid.x, currMax.y, mid.z), currNode->_children[2]);
    traverseRecursive(callback, Point(mid.x, mid.y, currMin.z),
                      Point(currMax.x, currMax.y, mid.z), currNode->_children[3]);
    traverseRecursive(callback, Point(currMin.x, currMin.y, mid.z),
                      Point(mid.x, mid.y, currMax.z), currNode->_children[4]);
    traverseRecursive(callback, Point(mid.x, currMin.y, mid.z),
                      Point(currMax.x, mid.y, currMax.z), currNode->_children[5]);
    traverseRecursive(callback, Point(currMin.x, mid.y, mid.z),
                      Point(mid.x, currMax.y, currMax.z), currNode->_children[6]);
    traverseRecursive(callback, mid, currMax, currNode->_children[7]);
  }
  void traverseNoRecursive(Callback* callback, const Point& currMin0, const Point& currMax0, OctreeNode* currNode0)
  {
    std::stack<Point> min_stack;
    std::stack<Point> max_stack;
    std::stack<OctreeNode*> node_stack;
    
    // ignore null pointer
    if (currNode0) {
      min_stack.push(currMin0);
      max_stack.push(currMax0);
      node_stack.push(currNode0);
    }
    
    while (!node_stack.empty()) {
      // get min, max and node
      Point currMin = min_stack.top();
      min_stack.pop();
      Point currMax = max_stack.top();
      max_stack.pop();
      OctreeNode* currNode = node_stack.top();
      node_stack.pop();
      
      bool shouldContinue = callback->operator()(currMin, currMax, currNode->_nodeData);
      if (!shouldContinue)
        continue;
      
      Point delta = currMax - currMin;
      Point mid = (delta * 0.5f) + currMin;
      
      // ignore null pointer
      if (currNode->_children[0]) {
        min_stack.push(currMin);
        max_stack.push(mid);
        node_stack.push(currNode->_children[0]);
      }
      
      // ignore null pointer
      if (currNode->_children[1]) {
        min_stack.push(Point(mid.x, currMin.y, currMin.z));
        max_stack.push(Point(currMax.x, mid.y, mid.z));
        node_stack.push(currNode->_children[1]);
      }
      
      // ignore null pointer
      if (currNode->_children[2]) {
        min_stack.push(Point(currMin.x, mid.y, currMin.z));
        max_stack.push(Point(mid.x, currMax.y, mid.z));
        node_stack.push(currNode->_children[2]);
      }
      
      // ignore null pointer
      if (currNode->_children[3]) {
        min_stack.push(Point(mid.x, mid.y, currMin.z));
        max_stack.push(Point(currMax.x, currMax.y, mid.z));
        node_stack.push(currNode->_children[3]);
      }
      
      // ignore null pointer
      if (currNode->_children[4]) {
        min_stack.push(Point(currMin.x, currMin.y, mid.z));
        max_stack.push(Point(mid.x, mid.y, currMax.z));
        node_stack.push(currNode->_children[4]);
      }
      
      // ignore null pointer
      if (currNode->_children[5]) {
        min_stack.push(Point(mid.x, currMin.y, mid.z));
        max_stack.push(Point(currMax.x, mid.y, currMax.z));
        node_stack.push(currNode->_children[5]);
      }
      
      // ignore null pointer
      if (currNode->_children[6]) {
        min_stack.push(Point(currMin.x, mid.y, mid.z));
        max_stack.push(Point(mid.x, currMax.y, currMax.z));
        node_stack.push(currNode->_children[6]);
      }
      
      // ignore null pointer
      if (currNode->_children[7]) {
        min_stack.push(mid);
        max_stack.push(currMax);
        node_stack.push(currNode->_children[7]);
      }
    }
  }
};

#endif


