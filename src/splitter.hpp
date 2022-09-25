//
//  splitter.hpp
//  crv_01
//
//  Created by Es1chUb on 2022/8/1.
//  Copyright © 2022 dhgir.abien. All rights reserved.
//

#ifndef splitter_h
#define splitter_h

#include "crpg.hpp"
#include "utility.hpp"
#include "interface.hpp"
#include "control_signal.hpp"

class State;

class Interval{
public:
    Interval(unsigned newLower, unsigned newUpper, unsigned newVarIndex):
        lower(newLower),
        upper(newUpper),
        var_index(newVarIndex)
    {}

    unsigned upper, lower;
    unsigned var_index;
};

class Node{
public:
    Node(std::vector<Interval> newRanges, Node* newParent):
        parent(newParent),
        rChild(nullptr),
        lChild(nullptr),
        num(0),
        entropy(0.0),
        varRanges(newRanges),
        violation(0),
        violation_set({0}),
        fsm_state(nullptr)
    {}

    Node*       parent;
    Node*       rChild;
    Node*       lChild;
    State*      state;
    unsigned    num;
    double      entropy;
    unsigned    violation;
    unsigned    piNum;
    std::set<unsigned>  violation_set;
    std::vector<Interval> varRanges;
    FsmState*   fsm_state;
    
    bool isLeaf(void){
        return (lChild == nullptr && rChild == nullptr)?true:false;
    }

    double estimateUCB(void){
        double ucb;

        if(num != 0 && parent){
            ucb = (entropy/num) + UCB_C * std::sqrt(std::log(parent->num)/num);
        }
        else{
            ucb = 1;
        }
        return ucb;
    }

    bool isPartitionable(void){
        for(unsigned i=0; i<varRanges.size(); i++){
            if(varRanges[i].lower < varRanges[i].upper){
                return true;
            }
        }
        return false;
    }
    
    bool isPartitionable(unsigned index){
        if(varRanges[index].lower < varRanges[index].upper){
            return true;
        }
        else{
            return false;
        }
    }
    
    bool isPartitionable(unsigned index, unsigned value){
        if((varRanges[index].lower <= value && value < varRanges[index].upper) ||
           (varRanges[index].lower < value && value < varRanges[index].upper) ){
            return true;
        }
        else{
            return false;
        }
    }
    
    std::vector<unsigned> genRegRandomPattern(Simulator* simulator){
        std::vector<unsigned> pattern;
        for(unsigned i=0; i<varRanges.size(); i++){
            pattern.emplace(pattern.end(), rGen.getRandNum(varRanges[i].lower, varRanges[i].upper));
        }
        return pattern;
    }
    
    std::vector<unsigned> genPiRandomPattern(Simulator* simulator){
        std::vector<unsigned> pattern;
        do{
            DPI::constraint_count++;
            DPI::rst_constraint_flag();
            pattern.clear();
            for(unsigned i=0; i<simulator->getPiNum(); i++){
                pattern.emplace(pattern.end(), rGen.getRandNum(0, simulator->getPiUpper(i)));
            }
            simulator->setPiPattern(pattern);
            simulator->eval();
        }while(DPI::get_constraint_flag());
        DPI::rst_constraint_flag();
        DPI::rst_asset_flag();
        return pattern;
    }
};

class Splitter{
public:
    Splitter(Simulator* newSimulator):
        simulator(newSimulator)
    {
        std::vector<Interval> rootRanges;
        for(unsigned i=0; i<simulator->getRegNum(); i++){
            rootRanges.emplace(
                    rootRanges.end(),
                    0,
                    simulator->getRegUpper(i),
                    i);
        }
        root = new Node(rootRanges, nullptr);
        assert = new Assertion();
        control = new ControlSignal();
    }

    Node* root;
    Simulator* simulator;
    Assertion* assert;
    ControlSignal* control;
    std::vector<Node*> abstractState;
    
    void updateAbstractState(Node* node){
        abstractState.clear();
        
        if(control->fsm_list.size() == 0){
            Node* tempNode = new Node(root->varRanges, nullptr);
            tempNode->fsm_state = new FsmState(0, 0b0, std::vector<unsigned>{0});
            abstractState.push_back(tempNode);
        }
        else{
            for(auto &s : control->fsm_list[0].states){
                Node* tempNode = new Node(root->varRanges, nullptr);
                tempNode->varRanges[control->fsm_list[0].varIndex].upper = s.encoding;
                tempNode->varRanges[control->fsm_list[0].varIndex].lower = s.encoding;
                tempNode->fsm_state = &s;
                abstractState.push_back(tempNode);
            }
        }
    }
    
    void printTree(Node* node){
        std::cout   << "addr: " << node <<std::endl
                    << "p_addr: " << node->parent << std::endl
                    << "l_addr: " << node->lChild << std::endl
                    << "r_addr: " << node->rChild << std::endl
                    << "num:" << node->num << " "
                    << "entropy:" << node->entropy << " "
                    << std::endl;
        for(unsigned i=0; i<node->varRanges.size(); i++){
            std::cout   << node->varRanges[i].var_index << " ["
                        << node->varRanges[i].lower << ","
                        << node->varRanges[i].upper << "]"
                        << std::endl;
        }
        std::cout << std::endl;
        
        if(node->lChild) printTree(node->lChild);
        if(node->rChild) printTree(node->rChild);
    }
    
    void printLeaf(Node* node){
        if(node->isLeaf()){
            std::cout   << "addr: " << node <<std::endl
                        << "p_addr: " << node->parent << std::endl
                        << "l_addr: " << node->lChild << std::endl
                        << "r_addr: " << node->rChild << std::endl
                        << "num:" << node->num << " "
                        << "entropy:" << node->entropy << " "
                        << std::endl;
            for(unsigned i=0; i<node->varRanges.size(); i++){
                std::cout   << node->varRanges[i].var_index << " ["
                            << node->varRanges[i].lower << ","
                            << node->varRanges[i].upper << "]"
                            << std::endl;
            }
            std::cout << std::endl;
        }
        
        if(node->lChild) printLeaf(node->lChild);
        if(node->rChild) printLeaf(node->rChild);
    }
};

#endif /* splitter_h */
