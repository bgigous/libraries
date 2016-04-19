// Copyright 2016 Carrie Rebhuhn
#ifndef STL_EASYSTL_H_
#define STL_EASYSTL_H_
#include <algorithm>

namespace easystl {
//! Clears using the swap idiom
template <class Container>
void clear(Container *q) {
    Container empty;
    std::swap(*q, empty);
}

//! Remove-erase-if idiom
template <class Container, class UnaryPredicate>
void remove_erase_if(Container stl, UnaryPredicate pred) {
    stl.erase(std::remove_if(stl.begin(), stl.end(), pred), stl.end());
}
}  // namespace easystl
#endif  // STL_EASYSTL_H_
