#ifndef QUEUE_H
#define QUEUE_H

#include <cstddef>


struct CQNode{
  CQNode(int CL, int CR, bool processing) : CL(CL), CR(CR), processing(processing), next(NULL) {}
  int CL, CR;
  bool processing;
  CQNode *next;
};


struct CQueue{
  CQueue() : first(NULL), last(NULL) {}

  void push(int CL, int CR, bool processing){
    CQNode *node = new CQNode(CL, CR, processing);
    if (!first){
      first = node;
      last = node;
    }
    else{
      last->next = node;
      last = node;
    }
  }
  void pop(){
    if (!first)  return;
    CQNode *temp = first;
    first = first->next;
    delete(temp);
  }
  bool empty(){
    return (first == NULL);
  }
  CQNode *first;
  CQNode *last;
};

#endif