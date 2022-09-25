//
//  main.cpp
//  Constrained Random Pattern Generation
//  dhgir.abien@gmail.com
//

#include "crpg.hpp"
#include "utility.hpp"
#include "simulator.hpp"
#include "interface.hpp"
#include "assertion.hpp"
#include "splitter.hpp"
#include "transition.hpp"
#include "analyzer.hpp"

#define PURE_RANDOM true
#define MCTS true

using namespace std;

int main(int argc, char** argv, char** env){
    timer.start();

    cout << "******************************"<< endl;
    cout << "*" << setw(16) << "CRPG" << setw(13) << "*" << endl;
    cout << "*" << setw(17) << "202208" << setw(12) << "*" << endl;
    cout << "******************************"<< endl;

    Simulator *simulator = new Simulator();
    
    cout << "******************************"<< endl;
    cout << "*" << setw(23) << "DUT Assertion List" << setw(6) << "*" << endl;
    cout << "******************************"<< endl;
    
    Assertion *assertion = new Assertion();
    cout << setiosflags(ios::left);
    cout << setw(10) << "Index" << setw(20) << "File" << setw(15) << "Line Number"<< "Property" << endl;
    for (auto &a : assertion->assertion){
        cout << setw(10) << a.index << setw(20) << a.fileName << setw(15) << a.lineNumber << '(' << a.statement << ')' << endl;
    }
    cout << resetiosflags(ios::left) << endl;
    cout << "Please select an index of assertion to generate counterexample: " << endl;
    
    if(argc > 1){
        DPI::assertion_active_index = stoi(argv[1]);
    }
    else{
        cin >> DPI::assertion_active_index;
    }
    while(DPI::assertion_active_index > assertion->assertion.size()-1){
        cout << "Out of range! Please select an index of assertion again: " << endl;
        cin >> DPI::assertion_active_index;
    }
    cout << "Assertion number " << DPI::assertion_active_index << " has been selected." << endl << endl;
    
    if(MCTS){
        cout << "******************************"<< endl;
        cout << "*" << setw(21) << "Abstract Model" << setw(8) << "*" << endl;
        cout << "******************************"<< endl;
        
        cout << "Building abstract state machine..." << endl << endl;
        Splitter *splitter = new Splitter(simulator);
        splitter->updateAbstractState(splitter->root);
        
        cout << "One-cycle random sampling...( " << VALID_SAMPLE_FACTOR*simulator->getRegNum() << " valid samples per state )" << endl;
        Transition *transition = new Transition(splitter->root, splitter->abstractState, simulator);
        if(!transition->createAbstractStateMachine(simulator->duv)){
            cout << "Can not get valid sample. The assertion holds." << endl << endl;
            return 0;
        }
        transition->printTransitionMatrix();
        
        double sampling_time = timer.getTime();
        
        cout << "******************************"<< endl;
        cout << "*" << setw(25) << "Extract CounterExample" << setw(4) << "*" << endl;
        cout << "******************************"<< endl;
        
        cout << "Extracting abstract execution sequence candidate..." << endl << endl;
        Analyzer *analyzer = new Analyzer(simulator, transition->abstractState, transition->root, transition->reg2State);
        analyzer->extractCounterExampleByMcts();
        cout << endl;
        
        cout << "******************************"<< endl;
        cout << "*" << setw(15) << "Result" << setw(14) << "*" << endl;
        cout << "******************************"<< endl;
        analyzer->printCounterExample();
        
        cout << "Elapsed time of sampling: " << sampling_time << " secs." << endl;
        cout << "Elapsed time of search: " << timer.getTime()-sampling_time << " secs." << endl;
        cout << "Elapsed total time: " << timer.getTime() << " secs." << endl;
        cout << "Total number of simulations: " << simulator->count << endl;
        cout << "Total number of Transition: " << transitionNum << endl;
        cout << "Total number of MCTS Node: " << mctsNodeNum << endl;
        cout << "Simulations per second: " << (unsigned)(simulator->count / timer.getTime()) << endl << endl;
    }
    double random_time = timer.getTime();
    if(PURE_RANDOM){
        cout << "******************************"<< endl;
        cout << "*" << setw(21) << "Pure Random" << setw(8) << "*" << endl;
        cout << "******************************"<< endl;
        
        Splitter *splitterR = new Splitter(simulator);
        State *stateR = new State(splitterR->root, 1, 1);
        simulator->resetDUV();
        DPI::rst_asset_flag();
        unsigned i;
        for(i=0; !DPI::get_asset_flag(); i++){
            std::vector<unsigned> piPatternR = stateR->genPiRandomPattern(simulator);
            simulator->setPiPattern(piPatternR);
            
            simulator->evalOneClock();
            if(i % 100000 == 1)
                printf("#%d\n", i-1);
            //printf("i_axi_ctrl=%u\n", simulator->duv->rootp->i_axi_ctrl);
            //printf("i_axi_ctrl_r=%u\n", simulator->duv->rootp->zipsystem__DOT__i_axi_ctrl_r);
            //printf("stall_count=%u\n", simulator->duv->rootp->zipsystem__DOT__thecpu__DOT__pf__DOT__stall_count);
            //printf("i_wb_stall=%u\n", simulator->duv->rootp->zipsystem__DOT__thecpu__DOT__pf__DOT__i_wb_stall);
            //printf("is_wb_active=%u\n", simulator->duv->rootp->zipsystem__DOT__thecpu__DOT__pf__DOT__is_wb_active);
            //printf("\n");
            //printf("i_d_rdata=%u\n", simulator->duv->rootp->i_d_rdata);
            //printf("*************************************************\n");
        }
        std::cout << "\nAssertion Violation in " << i << " step !!!" << std::endl;
        cout << "Elapsed time of search: " << timer.getTime()-random_time << " secs." << endl << endl;
        return 0;
    }

    
    return 0;
}
