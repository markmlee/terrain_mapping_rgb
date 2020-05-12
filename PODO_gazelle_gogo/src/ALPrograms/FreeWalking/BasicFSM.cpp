#include "BasicFSM.h"

using namespace std;

void RBAbstractFSM::Update(){
    CheckStateTransition();

    if(m_pCurrentState == NULL){
        if(m_pNewState != NULL){
            // set current state as new state
            m_pCurrentState = m_pNewState;
            m_pNewState = NULL;
            // start current state
            m_pCurrentState->ExecuteBeginState();
        }else{
            //cout << ">> m_pCurrentState is NULL..!!" << endl;
            return;
        }
    }
    if(m_pNewState != NULL){
        // finish current state
        m_pCurrentState->ExecuteEndState();
        // set current state as new state
        m_pCurrentState = m_pNewState;
        m_pNewState = NULL;
        // start current state
        m_pCurrentState->ExecuteBeginState();
    }
    // current state is working now
    if(m_pCurrentState != NULL)
        m_pCurrentState->ExecuteDoingState();
}

bool RBAbstractFSM::IsState(RBAState &state){
    if(m_pCurrentState == &state) return true;
    return false;
}

bool RBAbstractFSM::StateTransition(RBAState &newState){
    m_pNewState = &newState;
    return true;
}




