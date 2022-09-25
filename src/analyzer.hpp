//
//  analyzer.hpp
//  crv_01
//
//  Created by Es1chUb on 2022/8/2.
//  Copyright © 2022 dhgir.abien. All rights reserved.
//

#ifndef analyzer_h
#define analyzer_h

#include "crpg.hpp"
#include "utility.hpp"
#include "interface.hpp"
#include "splitter.hpp"
#include "transition.hpp"
#include "shortestpath.hpp"

class MctsNode{
public:
    MctsNode(MctsNode* newParent, State* newState, unsigned newLevel):
        parent(newParent),
        index(newState->index),
        state(newState),
        visits(0),
        score(0),
        reBranchCount(0),
        rsTree(nullptr),
        level(newLevel),
        operatorMomentum(false)
    {
        if(newState->rsTree != nullptr) rsTree = newState->rsTree->copy();
        value = newState->defaultValue;
    }
    
    MctsNode*               parent;
    std::list<MctsNode*>  children;
    
    unsigned index;
    unsigned visits;
    unsigned level;
    unsigned reBranchCount;
    double   score;
    double   value;
    State*   state;
    RsNode*  rsTree;
    bool     operatorMomentum;
    
    std::vector<unsigned> pi;
    std::vector<unsigned> reg;
    
    friend std::ostream& operator<< (std::ostream& os, const MctsNode& node){
        os << "index : " << node.index << std::endl
        << "visits: " << node.visits << std::endl
        << "score : " << node.score << std::endl
        << "value : " << node.value << std::endl;
        return os;
    }
    
    bool isLeaf(void){
        return children.empty();
    }
    
    bool isExpansible(void){
        return ((!operatorMomentum) && (children.size() < EXPANSION_FACTOR));
    }
    
    void attenuated(void){
        value = value * ATTENUATION_COEFFICIENT;
    }
    
    void updateValue(Simulator* simulator){
        if(state->inv_cov.shape().cols == 0){
            value = 0.0f;
            return;
        }
            
        std::vector<double> regPattern;
        simulator->getRegPattern(regPattern);
        
        nc::NdArray<double> x, x_mu, left, mahal, ans;
        x = nc::NdArray<double>(regPattern, false);
        
        x_mu = x - nc::mean(state->violation_snapshot, nc::Axis::ROW);
        left = nc::dot(x_mu, state->inv_cov);
        mahal = nc::dot(left, x_mu.transpose());
        ans = nc::diagonal(mahal);
        
        //std::cout.precision(8);
        //std::cout << ans[0]/(1.0+fabs(ans[0])) << std::endl;
        
        //value = (1/ans[0])*100;
        
        if(ans[0] > VALUE_NORMALIZATION_FACTOR || ans[0] < 0){
            value = 0.0f;
        }
        else{
            value = 1.0f - (ans[0] / VALUE_NORMALIZATION_FACTOR);
        }
        
    }
    
    void updateUCB(void){
        float valueSum = 0;
        float valueMax = 0;
        for(auto& child : children){
            valueSum += child->value;
            if(child->value < valueMax)
                valueMax = child->value;
        }
        
        float prior_score, value_score;
        if(parent){
            prior_score = UCB_C * std::sqrt(std::log(parent->visits)/visits);
        }
        else{
            prior_score = 0;
        }
        
        if(!children.empty()){
            value_score = valueSum/children.size();
            //value_score = valueMax;
        }
        else{
            value_score = value;
        }
        
        //std::cout << "(" << value_score << "+" << prior_score << ") ";
        if(value_score == 0){
            score = value_score;
        }
        else{
            score = value_score + prior_score;
        }
    }
    
    //********************************************************
    
    bool isReachable(void){
        return !pi.empty() || index == 0;
    }
    
    bool isNewBranch(void){
        if(reBranchCount > 5)
            return false;
        
        bool isDown = true;
        for(auto& child : children){
            if(child->value >= value){
                isDown = false;
                break;
            }
        }
        return isDown;
    }
    
    void updateAstarValue(Simulator* simulator){
        value = 0;
        float g = level;
        float h = 0;
        
        std::vector<unsigned> regPattern;
        simulator->getRegPattern(regPattern);
        
        h = regPattern[1];
        value = (h/5000);
    }
    
    RsNode* genPiPatternWithRsTree(Simulator* simulator, MctsNode* to, std::vector<unsigned> &pattern){
        DPI::constraint_count++;
        
        RsNode *node = rsTree;
        while(!node->isLeaf()){
            auto it_r = node->rChild->transitionTable.find(to->index);
            auto it_l = node->lChild->transitionTable.find(to->index);
            if(it_r == node->rChild->transitionTable.end() && it_l == node->lChild->transitionTable.end())
                break;
            if(it_l == node->lChild->transitionTable.end() || it_r->second > it_l->second){
                node = node->rChild;
            }
            else{
                node = node->lChild;
            }
        }
        
        for(unsigned i=0; i<simulator->getPiNum(); i++){
            pattern.emplace(pattern.end(), rGen.getRandNum(node->piRanges[i].lower, node->piRanges[i].upper));
        }
        return node;
    }
    
    void updateRsTreeTransitionTable(RsNode* curr, unsigned stateIndex){
        while(curr != nullptr){
            auto it = curr->transitionTable.find(index);
            if (it != curr->transitionTable.end()) {
                it->second++;
            } else {
                curr->transitionTable.insert({index, 1});
            }
            curr->transitionNum++;
            curr = curr->parent;
        }
    }
};

class Analyzer{
public:
    Analyzer(Simulator* newSimulator, std::vector<State*> newAbstractState, Node* newRoot, std::map<unsigned, State*> &reg2State):
        simulator(newSimulator),
        abstractState(newAbstractState),
        root(newRoot),
        reg2State(reg2State)
    {
        final_state = new State(nullptr, FINAL_STATE_INDEX, 0);
        
        control = new ControlSignal();
        if(control->fsm_list.size() != 0){
            controlRegIndex = control->fsm_list[0].varIndex;
        }
        else{
            controlRegIndex = -1;
        }
    }
    
    Node* root;
    Simulator* simulator;
    ControlSignal* control;
    std::vector<State*> abstractState;
    std::vector<std::vector<unsigned> > counterExample;
    std::vector<unsigned> counterExampleState;
    std::set<Node*> partitionCandidate;
    State *final_state;
    std::map<unsigned, State*> reg2State;
    int controlRegIndex;
    
    State* getNowStateBackup(void){
        for(auto &s : abstractState){
            bool flag = true;
            for(unsigned i=0; i<s->node->varRanges.size(); i++){
                if(!(s->node->varRanges[i].lower <= simulator->getREG(i)) ||
                   !(simulator->getREG(i) <= s->node->varRanges[i].upper) ){
                    flag = false;
                    break;
                }
            }
            if(flag){
                return s;
            }
        }
        return nullptr;
    }
    
    State* getNowState(void){
        if(controlRegIndex == -1){
            return abstractState[0];
        }
        else{
            return reg2State[simulator->getREG(controlRegIndex)];
        }
    }
    
    std::vector<unsigned> transferNode(MctsNode* from, MctsNode* to){
        std::vector<unsigned> regPattern;
        simulator->getRegPattern(regPattern);
        std::vector<unsigned> piPattern;
        simulator->getPiPattern(piPattern);
        RsNode* genRsNode = nullptr;
        
        // notion
        DPI::rst_stable_flag();
        
        for(unsigned t=0; t<STATE_TRANSFER_MAX_TRY && !DPI::get_asset_flag(); t++){
            transitionNum++;
            
            if(!DPI::get_stable_flag()){
                piPattern.clear();
                genRsNode = from->genPiPatternWithRsTree(simulator, to, piPattern);
                simulator->setPiPattern(piPattern);
            }

            //printSignal();
            DPI::rst_stable_flag();
            simulator->evalOneClock();
            State* stateNow = getNowState();
            //printSignal();
            
            if(genRsNode != nullptr){
                from->updateRsTreeTransitionTable(genRsNode, stateNow->index);
                from->state->updateTransitionTable(stateNow->index, 1);
            }
            if( stateNow->index == to->index || DPI::get_asset_flag()){
                return piPattern;
            }
            else{
                simulator->setRegPattern(regPattern);
                from->state->updateTransitionTable(to->state->index, -1);
            }
        }
        
        piPattern.clear();
        return piPattern;
    }
    
    unsigned getNextStateIndex(State *curr, std::set<unsigned>* nextStateSet){
        long long int rNum = rGen.getRandNum(0, curr->transitionSum);
        
        for(auto &index : *nextStateSet){
            if(index == FINAL_STATE_INDEX)
                return curr->index;
            rNum -= curr->transitionTable[index];
            if(rNum <= 0)
                return index;
        }
        
        // bug transitionSum not match
        return *nextStateSet->begin();
    }
    
    std::list<unsigned> getShortestPathOfState(unsigned from, unsigned to){
        Graph g;
        int parent[MAXV + 1];
        int distance[MAXV + 1];
        
        for(unsigned i=0; i<abstractState.size(); i++){
            for(unsigned j=0; j<abstractState[i]->transitionTable.size(); j++){
                if(i == j) continue;
                if(abstractState[i]->transitionTable[j] > 0){
                    g.insert_edge(abstractState[i]->index+1, j+1, 1, true);
                    //cout << abstractState[i]->index << " " << j << " " << abstractState[i]->transitionTable[j] << " " << endl;
                }
            }
        }
        
        dijkstra_shortest_path(&g, parent, distance, from+1);
        //print_shortest_path(to+1, parent);
        //print_distances(start, distance);
        //g.print();
        
        std::list<unsigned> shortestPath;
        get_shortest_path(to+1, parent, shortestPath);
        shortestPath.push_back(to);
        
        return shortestPath;
    }
    
    std::list<unsigned> getCandidatePathOfCounterExample(void){
        std::list<unsigned> path;
        for(auto& state: abstractState){
            //if(state->node->violation == 0 || state->node->violation == 2){
            if(state->node->violation == 2){
                path = getShortestPathOfState(0, state->index);
                if(path.size() > 1) break;
            }
        }
        return path;
    }
    
    bool extractCounterExampleByMcts(void){
        std::cout << "Searching Candidate of CounterExample by MCTS..." << std::endl;
        
        MctsNode* root = new MctsNode(nullptr, abstractState[0], 0);
        unsigned count = 0;
        unsigned simCount = 0;
        std::map<unsigned, std::set<unsigned>*> nextStateMap;
        std::map<unsigned, std::set<unsigned>*> nextCandidateMap;
        std::list<unsigned> path;
        unsigned preState = 0;
        bool operatorCachingFlag = false;
        std::vector<unsigned> operatorCachingPI;
        
        path = getCandidatePathOfCounterExample();
        if(path.size() > 1){
            std::cout << "New Candidate #" << count++ << ":" << std::endl;
            for(auto& p : path){
                std::cout << setw(5) << p;
            }
            std::cout << setw(5) << "!p" << std::endl;
            
            path.pop_front();
            for(auto& state : path){
                auto it = nextCandidateMap.find(preState);
                if(it != nextCandidateMap.end()){
                    it->second->insert(state);
                }
                else{
                    nextCandidateMap[preState] = new set<unsigned>{state};
                }
                preState = state;
            }
            nextCandidateMap[preState] = new set<unsigned>{FINAL_STATE_INDEX};
        }
        for(auto &s : abstractState){
            nextStateMap[s->index] = new set<unsigned>;
            for(unsigned i=0; i < s->transitionTable.size(); i++){
                if(s->transitionTable[i] > 0){
                    nextStateMap[s->index]->insert(i);
                }
            }
        }
        
        //DPI::rst_asset_flag();
        while(true){
            //std::cout << "#" << simCount++ << std::endl;
            
            //Selection
            MctsNode* curr = root;
            std::cout << "Selection:";
            DPI::rst_asset_flag();
            simulator->resetDUV();
            while(!curr->isExpansible()){
                MctsNode* next = curr->children.front();
                unsigned idx = 0;
                unsigned nextIdx = 0;
                for(auto& child : curr->children){
                    if(next->score < child->score){
                        next = child;
                        nextIdx = idx;
                    }
                    idx++;
                }
                simulator->setPiPattern(next->pi);
                simulator->evalOneClock();
                std::cout << nextIdx << " ";
                //printf("\ni_axi_ctrl=%u\n", simulator->duv->rootp->i_axi_ctrl);
                //printf("i_axi_ctrl_r=%u\n", simulator->duv->rootp->zipsystem__DOT__i_axi_ctrl_r);
                //printf("mmu i_axi_ctrl=%u\n", simulator->duv->rootp->zipsystem__DOT__themmu__DOT__i_axi_ctrl);
                //printf("mmu o_rtn_stall=%u\n", simulator->duv->rootp->zipsystem__DOT__themmu__DOT__o_rtn_stall);
                //printf("pf i_wb_stall=%u\n", simulator->duv->rootp->zipsystem__DOT__thecpu__DOT__pf__DOT__i_wb_stall);
                //printf("pf stall_count=%u\n", simulator->duv->rootp->zipsystem__DOT__thecpu__DOT__pf__DOT__stall_count);
                //printf("asset_flag=%d\n\n", DPI::get_asset_flag());

                curr = next;
            }
            std::cout << std::endl << "Step: " << curr->level << std::endl;
            
            //Expansion
            //std::cout << "Expansion: ";
//#TODO
            unsigned newIndex;
            if(nextCandidateMap.find(curr->index) != nextCandidateMap.end())
                newIndex = getNextStateIndex(curr->state, nextCandidateMap[curr->index]);
            else
                newIndex = getNextStateIndex(curr->state, nextStateMap[curr->index]);
                
            MctsNode* newNode = new MctsNode(curr, abstractState[newIndex], curr->level+1);
            mctsNodeNum++;
            curr->children.push_back(newNode);
            //std::cout << setw(10) << newNode->index << std::endl;
            
            //Simulation
            //std::cout << "Simulation";
            
            std::vector<unsigned> in;
            // Operator Momentum Caching
            if(operatorCachingFlag){
                in = operatorCachingPI;
                std::cout << "Operator Momentum Caching" << std::endl;
            }
            else{
                in = transferNode(curr, newNode);
            }
            if(in.empty()){
                //std::cout << " failure";
                auto it = curr->children.begin();
                for(; *it != newNode; it++)
                    continue;
                curr->children.erase(it);
                
                newNode = new MctsNode(curr, abstractState[curr->state->index], curr->level+1);
                curr->children.push_back(newNode);
                std::vector<unsigned> in = transferNode(curr, newNode);
                //curr->attenuated();
            }
            
            //std::cout << " success";
            newNode->pi = in;
            newNode->updateValue(simulator);
            //std::cout << "pixel_index: " << simulator->duv->rootp->cle__DOT__pixel_index << std::endl;
            
            //std::cout << std::endl;
            
            //Operator Momentum Caching
            if(newNode->value > newNode->parent->value){
                operatorCachingFlag = true;
                operatorCachingPI = in;
                newNode->parent->operatorMomentum = true;
            }
            else{
                operatorCachingFlag = false;
            }
            
            // Check Violation
            if(DPI::get_asset_flag()){
                std::cout << "Violation: " << DPI::assertion_index << std::endl;
                MctsNode* temp = newNode;
                while(temp->parent != nullptr){
                    counterExample.push_back(temp->pi);
                    counterExampleState.push_back(temp->index);
                    temp = temp->parent;
                }
                std::reverse(counterExample.begin(), counterExample.end());
                std::reverse(counterExampleState.begin(), counterExampleState.end());
                return true;
            }
            else{
                //Backpropagation
                std::cout << "Backpropagation: ";
                MctsNode* temp = newNode;
                while(temp != nullptr){
                    temp->visits++;
                    temp->updateUCB();
                    std::cout << "(" << temp->value << ", " << temp->score << ") ";
                    temp = temp->parent;
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
            std::cout << std::endl << "*************************" << std::endl;
        }
    }
    
    void printCounterExample(void){
        if(counterExample.size() > 0){
            std::cout << "CounterExample is found as follows:" << std::endl;
            
            std::cout << std::setw(11) << " ";
            for(unsigned i=0; i<simulator->getPiNum(); i++){
                std::cout << std::setw(12) << simulator->getPiSignal(i)->name << " ";
            }
            std::cout << std::endl;
            
            simulator->resetDUV();
            DPI::rst_asset_flag();
            
            unsigned i;
            for(i=0; i<counterExample.size(); i++){
                std::cout << std::setw(6) << "#"+std::to_string(i) << " ";
                if(counterExampleState[i] == 4294967295)
                    std::cout << std::setw(4) << "(!p)";
                else
                    std::cout << std::setw(4) << "("+std::to_string(counterExampleState[i])+")";
                for(auto &v : counterExample[i]){
                    std::cout << std::setw(12) << v << " ";
                }
                simulator->setPiPattern(counterExample[i]);
                simulator->evalOneClock();
                //simulator->printSignal();
                std::cout << std::endl;
            }
            while(!DPI::get_asset_flag()){
                    simulator->evalOneClock();
                    std::cout << std::setw(6) << "#"+std::to_string(i++) << std::endl;
            }

            std::cout << std::setw(6) << "#"+std::to_string(i);
            std::cout << "  Assertion Violation!!!" << std::endl;
            cout << endl << "******************************"<< endl;

            if(DPI::get_asset_flag()){
                cout << "*" << "    CounterExample Valid    " << "*" << endl;
                cout << "*" << "    Assertion no. " << std::setw(5) << DPI::assertion_index << "     *" << endl;
            }
            else{
                cout << "*" << "    CounterExample Invalid. " << "*" << endl;
            }
            cout << "******************************"<< endl << endl;
        }
        else{
            std::cout << "Could not find counterexample." << std::endl;
        }
    }
};

#endif /* analyzer_h */
