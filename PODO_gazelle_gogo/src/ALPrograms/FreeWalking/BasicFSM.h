#ifndef BASICFSM_H
#define BASICFSM_H

#include <stdio.h>
#include <iostream>


class RBAbstractState;
class RBAbstractFSM;

typedef RBAbstractState		RBAState;
typedef RBAbstractFSM		RBAFSM;
typedef	RBAState*			pRBAState;
typedef RBAFSM*				pRBAFSM;


class RBAbstractState{
public:
    RBAbstractState(RBAFSM *parent = 0)
        : parentFSM(parent) {}

    // execute functions (pure virtua function == user has to make these implementation)
    virtual void	ExecuteBeginState(void) = 0;
    virtual void	ExecuteDoingState(void) = 0;
    virtual void	ExecuteEndState(void) = 0;
protected:
    pRBAFSM		parentFSM;
};


class RBAbstractFSM{
protected:
    pRBAState	m_pCurrentState;
    pRBAState	m_pNewState;

public:
    RBAbstractFSM()
        : m_pCurrentState(NULL), m_pNewState(NULL) {}
    virtual ~RBAbstractFSM() {}

    // update
    virtual void Update(void);
    // current state check
    bool		IsState(RBAState &state);
    // state transition
    bool		StateTransition(RBAState &newState);

    // state transition checker (pure virtual function == user has to make these implementation)
    virtual void CheckStateTransition(void) = 0;
};


#endif // BASICFSM_H
