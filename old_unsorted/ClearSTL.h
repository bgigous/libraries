#pragma once

#include <typeinfo>

/*
| A bunch of handy shortcuts for clearing out stl containers with pointers.
*/

// AVAILABLE UP FRONT

template <class T>
void destroy1(T toDestroy){
    if (toDestroy==NULL) return;
    /*
    | Inputs: A pointer to the container, the first reverse iterator for the container (rbegin).
    */
    for (typeof(toDestroy->rbegin()) outer=toDestroy->rbegin();outer!=toDestroy->rend();++outer){
        // TODO: TYPEID TO CATCH WHEN A 2D VECTOR OR A MAP USED??
        delete *outer;
    }
    toDestroy->clear();
    delete toDestroy;
}

template <class T>
void destroy2(T toDestroy){
    if (toDestroy==NULL) return;
    for (typeof(toDestroy->rbegin()) outer=toDestroy->rbegin(); outer!=toDestroy->rend(); ++outer){
        for (typeof((*outer)->rbegin()) inner=(*outer)->rbegin(); inner!=(*outer)->rend(); ++inner){
            delete *inner;
        }
    }
    toDestroy->clear();
    delete toDestroy;
}

template <class T>
void destroyOneToMany(T toDestroy){
    if (toDestroy==NULL) return;
    for (typeof(toDestroy->rbegin()) outer=toDestroy->rbegin(); outer!=toDestroy->rend(); ++outer){
        for (typeof(outer->second->rbegin()) inner=outer->second->rbegin(); inner!=outer->second->rend(); ++inner){
            delete *inner;
        }
        delete *outer->first;
    }
    toDestroy->clear();
    delete toDestroy;
}

template <class T>
void destroyObjToPtr(T toDestroy){
    if (toDestroy==NULL) return;
    for (typeof(toDestroy->rbegin()) outer=toDestroy->rbegin(); outer!=toDestroy->rend(); ++outer){
        delete outer->second;
    }
    toDestroy->clear();
    delete toDestroy;
}
