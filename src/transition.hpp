//
//  transition.hpp
//  crv_01
//
//  Created by Es1chUb on 2022/8/1.
//  Copyright © 2022 dhgir.abien. All rights reserved.
//

#ifndef transition_h
#define transition_h

#include "crpg.hpp"
#include "utility.hpp"
#include "interface.hpp"
#include "splitter.hpp"
#include "cart.hpp"

class RsNode{
public:
    RsNode(std::vector<Interval> newRanges, RsNode* newParent):
        parent(newParent),
        rChild(nullptr),
        lChild(nullptr),
        piRanges(newRanges),
        transitionNum(0)
    {}
    
    RsNode*       parent;
    RsNode*       rChild;
    RsNode*       lChild;
    std::vector<Interval> piRanges;
    std::map<unsigned, unsigned> transitionTable;
    unsigned transitionNum;
    
    bool isLeaf(void){
        return (lChild == nullptr && rChild == nullptr)?true:false;
    }
    
    bool isPartitionable(unsigned index, unsigned value){
        if((piRanges[index].lower <= value && value < piRanges[index].upper) ||
           (piRanges[index].lower < value && value < piRanges[index].upper) ){
            return true;
        }
        else{
            return false;
        }
    }
    
    RsNode* copy(void){
        RsNode* newNode = new RsNode(piRanges, parent);
        newNode->transitionTable = transitionTable;
        newNode->transitionNum = transitionNum;
        if(!isLeaf()){
            newNode->rChild = rChild->copy();
            newNode->rChild->parent = newNode;
            newNode->lChild = lChild->copy();
            newNode->lChild->parent = newNode;
        }
        return newNode;
    }
};

class Snapshot{
public:
    Snapshot(
        unsigned newIndex,
        std::vector<unsigned> newReg,
        std::vector<unsigned> newPi
    ):
        index(newIndex),
        reg(newReg),
        pi(newPi)
    {}
    
    unsigned index;
    std::vector<unsigned> reg;
    std::vector<unsigned> pi;
};

class State{
public:
    State(Node* newNode, unsigned newIndex, unsigned stateNum):
        node(newNode),
        index(newIndex),
        rsTree(nullptr),
        transitionSum(0),
        defaultValue(0),
        encoding(0)
    {
        std::vector<unsigned> vec( stateNum , 0 );
        transitionTable = vec;
    }
    
    Node* node;
    unsigned index;
    unsigned defaultValue;
    unsigned encoding;
    RsNode* rsTree;
    unsigned transitionSum;
    std::vector<unsigned> transitionTable;
    nc::NdArray<double> violation_snapshot;
    nc::NdArray<double> cov;
    nc::NdArray<double> inv_cov;
    
    std::vector<unsigned> genRegRandomPattern(Simulator* simulator){
        return node->genRegRandomPattern(simulator);
    }
    
    std::vector<unsigned> genPiRandomPattern(Simulator* simulator){
        return node->genPiRandomPattern(simulator);
    }
    
    std::vector<unsigned> genPiRandomPatternWithRsTree(Simulator* simulator, State *stateTo){
        DPI::constraint_count++;
        
        RsNode *node = rsTree;
        while(!node->isLeaf()){
            auto it_r = node->rChild->transitionTable.find(stateTo->index);
            auto it_l = node->lChild->transitionTable.find(stateTo->index);
            if(it_r->second > it_l->second){
                node = node->rChild;
            }
            else{
                node = node->lChild;
            }
        }
        
        std::vector<unsigned> pattern;
        for(unsigned i=0; i<simulator->getPiNum(); i++){
            pattern.emplace(pattern.end(), rGen.getRandNum(node->piRanges[i].lower, node->piRanges[i].upper));
        }
        return pattern;
    }
    
    void updateTransitionTable(unsigned index, int num){
        if(((int)transitionTable[index])+num > 1){
            transitionTable[index] += num;
            transitionSum += num;
        }
    }
};

class Transition{
public:
    Transition(Node* newRoot, std::vector<Node*> newAbstractState, Simulator* newSimulator):
        root(newRoot),
        initState(nullptr),
        simulator(newSimulator)
    {
        assert = new Assertion();
        control = new ControlSignal();
        
        for(unsigned i=0; i<newAbstractState.size(); i++){
            State *temp = new State(newAbstractState[i], i, newAbstractState.size());
            abstractState.push_back(temp);
            newAbstractState[i]->state = abstractState[i];
            for(auto& idx : abstractState[i]->node->fsm_state->transitionTable){
                abstractState[i]->transitionTable[idx] = 1;
                abstractState[i]->transitionSum++;
            }
            if(control->fsm_list.size() != 0){
                abstractState[i]->encoding = control->fsm_list[0].states[i].encoding;
                reg2State[abstractState[i]->encoding] = abstractState[i];
            }
        }
        if(control->fsm_list.size() != 0){
            controlRegIndex = control->fsm_list[0].varIndex;
        }
        else{
            controlRegIndex = -1;
        }
    }
    
    Node* root;
    State* initState;
    Assertion* assert;
    Simulator* simulator;
    ControlSignal* control;
    std::vector<State*> abstractState;
    std::map<unsigned, State*> reg2State;
    int controlRegIndex;
    
    State* getNowState(void){
        if(controlRegIndex == -1){
            return abstractState[0];
        }
        else{
            return reg2State[simulator->getREG(controlRegIndex)];
        }
    }
    
    void createInputRsTree(State *state){
        std::vector<Interval> rootRanges;
        for(unsigned i=0; i<simulator->getPiNum(); i++){
            rootRanges.emplace(
                    rootRanges.end(),
                    0,
                    simulator->getPiUpper(i),
                    i);
        }
        state->rsTree = new RsNode(rootRanges, nullptr);
    }
    
    void updateInputRsTreeRecursive(State *state, Solution &solu, RsNode *node, unsigned treeIndex){
        if (solu.tree[treeIndex].nodeType == DTNode::NODE_INTERNAL){
            if(solu.params->attributeTypes[solu.tree[treeIndex].splitAttribute] == TYPE_NUMERICAL){
                node->lChild = new RsNode(node->piRanges, node);
                node->rChild = new RsNode(node->piRanges, node);
                node->lChild->piRanges[solu.tree[treeIndex].splitAttribute].upper = floor(solu.tree[treeIndex].splitValue);
                node->rChild->piRanges[solu.tree[treeIndex].splitAttribute].lower = solu.tree[treeIndex].splitValue+1;
                updateInputRsTreeRecursive(state, solu, node->lChild, 2*treeIndex+1);
                updateInputRsTreeRecursive(state, solu, node->rChild, 2*treeIndex+2);
            }
            else{
                node->rChild = new RsNode(node->piRanges, node);
                node->piRanges[solu.tree[treeIndex].splitAttribute].upper = floor(solu.tree[treeIndex].splitValue);
                node->piRanges[solu.tree[treeIndex].splitAttribute].lower = floor(solu.tree[treeIndex].splitValue);
                node->lChild = new RsNode(node->piRanges, node);
                updateInputRsTreeRecursive(state, solu, node->lChild, 2*treeIndex+1);
                updateInputRsTreeRecursive(state, solu, node->rChild, 2*treeIndex+2);
            }
        }
        else if (solu.tree[treeIndex].nodeType == DTNode::NODE_LEAF){
            while(node != nullptr){
                auto it = node->transitionTable.find(solu.tree[treeIndex].majorityClass);
                if (it != node->transitionTable.end()) {
                    it->second += solu.tree[treeIndex].nbSamplesClass[solu.tree[treeIndex].majorityClass];
                } else {
                    node->transitionTable.insert({solu.tree[treeIndex].majorityClass, solu.tree[treeIndex].nbSamplesClass[solu.tree[treeIndex].majorityClass]});
                }
                node->transitionNum += solu.tree[treeIndex].nbSamplesClass[solu.tree[treeIndex].majorityClass];
                node = node->parent;
            }
        }
    }
    
    void updateStateTransitionTable(State *state, State *nowState){
        if(state->transitionTable[nowState->index] != 0)
            state->transitionTable[nowState->index]++;
    }
    
    bool createAbstractStateMachine(std::unique_ptr<Vdesign_under_test> &duv){
        // set init state
        simulator->resetDUV();
        initState = getNowState();
        bool violation_flag = false;
        unsigned valid_sample_num =  VALID_SAMPLE_FACTOR*simulator->getRegNum();
        
        for(auto &state : abstractState){
            std::cout << "----- STATE SAMPLING ON STATE" << state->index << std::endl;
            double sampling_start_time = timer.getTime();
            
            unsigned valid_sample_count;
            bool violation_state_flag = false;
            state->violation_snapshot = nc::NdArray<double>(valid_sample_num, simulator->getRegNum());
            std::vector<std::vector<double> > dataset;
            std::vector<int> datasetClasses;
            
            createInputRsTree(state);
            
            for(state->transitionSum=0, valid_sample_count=0;
                (valid_sample_count < valid_sample_num) &&
                ( violation_state_flag || (timer.getTime()-sampling_start_time < SAMPLE_TIMEOUT) );
                state->transitionSum++){
                std::vector<unsigned> regPattern = state->genRegRandomPattern(simulator);
                simulator->setRegPattern(regPattern);
                
                //simulator->printSignal();
                
                std::vector<unsigned> piPattern = state->genPiRandomPattern(simulator);
                simulator->setPiPattern(piPattern);
                dataset.push_back(std::vector<double>(piPattern.begin(), piPattern.end()));
                
                //for(auto &n : regPattern) std::cout << n << " "; std::cout << std::endl;
                //for(auto &n : piPattern) std::cout << n << " "; std::cout << std::endl << std::endl;
                
                simulator->evalOneClock();
                
                State *nowState = getNowState();
                updateStateTransitionTable(state, nowState);
                datasetClasses.push_back(nowState->index);
                
                if( DPI::get_asset_flag() ){
                    //std::cout << state->index << std::endl;
                    //for(auto &v : regPattern) std::cout << v << " "; std::cout << std::endl;
                    violation_flag = violation_state_flag = true;
                    state->node->violation = 2;
                    
                    //std::vector<double> tempPattern(regPattern.begin(), regPattern.end());
                    //nc::NdArray<double> temp = nc::NdArray<double>(tempPattern, false);
                    
                    if(valid_sample_count != 0){
                        for(unsigned k=0; k<regPattern.size(); k++){
                            state->violation_snapshot.put(valid_sample_count*regPattern.size()+k ,regPattern[k]);
                        }
                    }
                    else{
                        for(unsigned k=0; k<regPattern.size(); k++){
                            state->violation_snapshot.put(valid_sample_count*regPattern.size()+k ,regPattern[k]+INV_MATRIX_DELTA);
                        }
                    }
                    valid_sample_count++;
                    
                    if( valid_sample_count % 100 == 0)
                        std::cout << "A total of " << std::setw(6) << valid_sample_count
                            << " valid samples were obtained after " << std::setw(6) << state->transitionSum << " simulations" << std::endl;
                }
                
                simulator->resetDUV();
                DPI::rst_asset_flag();
            }
            std::cout << "----- STATE SAMPLING COMPLETED IN " << (timer.getTime() - sampling_start_time)<< "(s)" << std::endl;
            
            if(valid_sample_count > 0){
                std::cout << "----- STARTING ESTIMATE THE COVARIANCE MATRIX" << std::endl;
                double cov_start_time = timer.getTime();
                
                std::cout << "State " << state->index << " violation_snapshot: " << state->violation_snapshot.shape();
                //std::cout << state->violation_snapshot << std::endl << std::endl;

                //state->violation_snapshot = nc::deleteIndices(state->violation_snapshot, {1}, nc::Axis::COL);
                //std::cout << state->violation_snapshot << std::endl << std::endl;

                state->cov = nc::cov(state->violation_snapshot.transpose());
                //std::cout << "state->cov" << std::endl << std::endl;
                //std::cout << state->cov << std::endl << std::endl;
                state->inv_cov = nc::linalg::inv(state->cov);
                //std::cout << "state->inv_cov" << std::endl << std::endl;
                //std::cout << state->inv_cov << std::endl << std::endl;
                std::cout << "----- ESTIMATE THE COVARIANCE MATRIX COMPLETED IN " << (timer.getTime() - cov_start_time)<< "(s)" << std::endl;
            }
            else{
                //todo
                ;
            }
            
            std::vector<std::string> attributes;
            for(unsigned k=0; k<simulator->getPiNum(); k++){
                if(simulator->getPiWidth(k) > 4)
                    attributes.push_back("N");
                else{
                    attributes.push_back("C");
                }
            }
            
            if(abstractState.size() > 1){
                Params params(dataset, attributes, datasetClasses,
                              rGen.getRandNum(0, 1024),
                              CART_DEPTH,
                              CART_TIMEOUT * CLOCKS_PER_SEC,
                              std::to_string(state->index),
                              abstractState.size());
                Solution solution(&params);
                std::cout << "----- STARTING DECISION TREE OPTIMIZATION" << std::endl;
                double dt_start_time = timer.getTime();
                Greedy solver(&params,&solution);
                solver.run();
                std::cout << "----- DECISION TREE OPTIMIZATION COMPLETED IN " << (timer.getTime() - dt_start_time) << "(s)" << std::endl;
                solution.printAndExport();
                std::cout << "----- END OF DECISION TREE ALGORITHM" << std::endl << std::endl;
                
                updateInputRsTreeRecursive(state, solution, state->rsTree, 0);
                //printRsTree(state->rsTree);
            }
        }
        std::cout << std::endl;
        return violation_flag;
    }
    
    void printTransitionState(void){
        for(unsigned i = 0; i < abstractState.size(); i++){
            Node* node = abstractState[i]->node;
            std::cout   << "index: " << abstractState[i]->index << std::endl
                        << "address: " << node << std::endl
                        << "sim_num:" << node->num << " " << std::endl
                        << "entropy:" << node->entropy << " " << std::endl
                        << "violation:";
                        if(node->violation == 0) std::cout << "p & !p";
                        else if(node->violation == 1) std::cout << "p ";
                        else if(node->violation == 2) std::cout << "!p ";
                        std::cout << std::endl;
            for(unsigned i=0; i<node->varRanges.size(); i++){
                std::cout   << std::setw(3) << node->varRanges[i].var_index << " "
                            << std::setw(20) << simulator->getRegSignal(i)->name << " ["
                            << node->varRanges[i].lower << ","
                            << node->varRanges[i].upper << "]"
                            << std::endl;
            }
            std::cout << std::endl;
            
            std::cout   << "RS Tree: " << abstractState[i]->rsTree->transitionTable.size() << std::endl;
            for(auto& pair : abstractState[i]->rsTree->transitionTable){
                std::cout << pair.first << " " << pair.second << std::endl;
            }
            std::cout << std::endl;
        }
    }
    
    void printTransitionMatrix(void){
        std::cout   << std::endl << "Transition Probability Matrix " << abstractState.size() << " x " << abstractState.size() << std::endl;
        std::cout << std::setw(10) << " ";
        for(unsigned i = 0; i < abstractState.size(); i++){
            std::cout <<std::setw(10) << "#"+std::to_string(i);
        }
        std::cout << std::endl;
        
        for(unsigned i = 0; i < abstractState.size(); i++){
            std::string temp;
            if(initState == abstractState[i]) temp+="* ";
            if(abstractState[i]->node->violation==1) temp+="(p) ";
            else if(abstractState[i]->node->violation==2) temp+="(!p) ";
            std::cout << std::setw(10) << temp+"#"+std::to_string(i);
            std::cout.precision(2);
            double sum = 0;
            for(unsigned j = 0; j < abstractState[i]->transitionTable.size(); j++){
                sum += abstractState[i]->transitionTable[j];
            }
            for(unsigned j = 0; j < abstractState[i]->transitionTable.size(); j++){
                std::cout << std::setw(10) << ((double)abstractState[i]->transitionTable[j]/sum);
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout.precision(4);
    }
    
    void printRsTree(RsNode* node){
        std::cout   << "addr: " << node <<std::endl
                    << "p_addr: " << node->parent << std::endl
                    << "l_addr: " << node->lChild << std::endl
                    << "r_addr: " << node->rChild << std::endl
                    << "num:" << node->transitionNum << " "
                    << std::endl;
        for(unsigned i=0; i<node->piRanges.size(); i++){
            std::cout   << node->piRanges[i].var_index << " ["
                        << node->piRanges[i].lower << ","
                        << node->piRanges[i].upper << "]"
                        << std::endl;
        }
        std::cout << std::endl;
        
        if(node->lChild) printRsTree(node->lChild);
        if(node->rChild) printRsTree(node->rChild);
    }
};

#endif /* transition_h */
