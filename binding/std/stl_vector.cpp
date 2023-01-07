#include <iterator>
#include <memory>
#include <sstream> // __str__
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include "actuatordefine.h"


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_std_stl_vector(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::vector file:bits/stl_vector.h line:389
		pybind11::class_<std::vector<unsigned short>, std::shared_ptr<std::vector<unsigned short>>> cl(M("std"), "vector_unsigned_short_t", "");
		cl.def( pybind11::init( [](){ return new std::vector<unsigned short>(); } ) );
		cl.def( pybind11::init<const class std::allocator<unsigned short> &>(), pybind11::arg("__a") );

		cl.def( pybind11::init( [](unsigned long const & a0){ return new std::vector<unsigned short>(a0); } ), "doc" , pybind11::arg("__n"));
		cl.def( pybind11::init<unsigned long, const class std::allocator<unsigned short> &>(), pybind11::arg("__n"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](unsigned long const & a0, const unsigned short & a1){ return new std::vector<unsigned short>(a0, a1); } ), "doc" , pybind11::arg("__n"), pybind11::arg("__value"));
		cl.def( pybind11::init<unsigned long, const unsigned short &, const class std::allocator<unsigned short> &>(), pybind11::arg("__n"), pybind11::arg("__value"), pybind11::arg("__a") );

		cl.def( pybind11::init( [](std::vector<unsigned short> const &o){ return new std::vector<unsigned short>(o); } ) );
		cl.def( pybind11::init<const class std::vector<unsigned short, class std::allocator<unsigned short> > &, const class std::allocator<unsigned short> &>(), pybind11::arg("__x"), pybind11::arg("__a") );

		cl.def("assign", (class std::vector<unsigned short, class std::allocator<unsigned short> > & (std::vector<unsigned short>::*)(const class std::vector<unsigned short, class std::allocator<unsigned short> > &)) &std::vector<unsigned short>::operator=, "C++: std::vector<unsigned short>::operator=(const class std::vector<unsigned short, class std::allocator<unsigned short> > &) --> class std::vector<unsigned short, class std::allocator<unsigned short> > &", pybind11::return_value_policy::automatic, pybind11::arg("__x"));
		cl.def("assign", (void (std::vector<unsigned short>::*)(unsigned long, const unsigned short &)) &std::vector<unsigned short>::assign, "C++: std::vector<unsigned short>::assign(unsigned long, const unsigned short &) --> void", pybind11::arg("__n"), pybind11::arg("__val"));
		cl.def("begin", (class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::begin, "C++: std::vector<unsigned short>::begin() --> class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >");
		cl.def("end", (class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::end, "C++: std::vector<unsigned short>::end() --> class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >");
		cl.def("cbegin", (class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)() const) &std::vector<unsigned short>::cbegin, "C++: std::vector<unsigned short>::cbegin() const --> class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >");
		cl.def("cend", (class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)() const) &std::vector<unsigned short>::cend, "C++: std::vector<unsigned short>::cend() const --> class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >");
		cl.def("size", (unsigned long (std::vector<unsigned short>::*)() const) &std::vector<unsigned short>::size, "C++: std::vector<unsigned short>::size() const --> unsigned long");
		cl.def("max_size", (unsigned long (std::vector<unsigned short>::*)() const) &std::vector<unsigned short>::max_size, "C++: std::vector<unsigned short>::max_size() const --> unsigned long");
		cl.def("resize", (void (std::vector<unsigned short>::*)(unsigned long)) &std::vector<unsigned short>::resize, "C++: std::vector<unsigned short>::resize(unsigned long) --> void", pybind11::arg("__new_size"));
		cl.def("resize", (void (std::vector<unsigned short>::*)(unsigned long, const unsigned short &)) &std::vector<unsigned short>::resize, "C++: std::vector<unsigned short>::resize(unsigned long, const unsigned short &) --> void", pybind11::arg("__new_size"), pybind11::arg("__x"));
		cl.def("shrink_to_fit", (void (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::shrink_to_fit, "C++: std::vector<unsigned short>::shrink_to_fit() --> void");
		cl.def("capacity", (unsigned long (std::vector<unsigned short>::*)() const) &std::vector<unsigned short>::capacity, "C++: std::vector<unsigned short>::capacity() const --> unsigned long");
		cl.def("empty", (bool (std::vector<unsigned short>::*)() const) &std::vector<unsigned short>::empty, "C++: std::vector<unsigned short>::empty() const --> bool");
		cl.def("reserve", (void (std::vector<unsigned short>::*)(unsigned long)) &std::vector<unsigned short>::reserve, "C++: std::vector<unsigned short>::reserve(unsigned long) --> void", pybind11::arg("__n"));
		cl.def("__getitem__", (unsigned short & (std::vector<unsigned short>::*)(unsigned long)) &std::vector<unsigned short>::operator[], "C++: std::vector<unsigned short>::operator[](unsigned long) --> unsigned short &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("at", (unsigned short & (std::vector<unsigned short>::*)(unsigned long)) &std::vector<unsigned short>::at, "C++: std::vector<unsigned short>::at(unsigned long) --> unsigned short &", pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def("front", (unsigned short & (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::front, "C++: std::vector<unsigned short>::front() --> unsigned short &", pybind11::return_value_policy::automatic);
		cl.def("back", (unsigned short & (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::back, "C++: std::vector<unsigned short>::back() --> unsigned short &", pybind11::return_value_policy::automatic);
		cl.def("data", (unsigned short * (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::data, "C++: std::vector<unsigned short>::data() --> unsigned short *", pybind11::return_value_policy::automatic);
		cl.def("push_back", (void (std::vector<unsigned short>::*)(const unsigned short &)) &std::vector<unsigned short>::push_back, "C++: std::vector<unsigned short>::push_back(const unsigned short &) --> void", pybind11::arg("__x"));
		cl.def("pop_back", (void (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::pop_back, "C++: std::vector<unsigned short>::pop_back() --> void");
		cl.def("insert", (class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >, const unsigned short &)) &std::vector<unsigned short>::insert, "C++: std::vector<unsigned short>::insert(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >, const unsigned short &) --> class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >", pybind11::arg("__position"), pybind11::arg("__x"));
		cl.def("insert", (class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >, unsigned long, const unsigned short &)) &std::vector<unsigned short>::insert, "C++: std::vector<unsigned short>::insert(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >, unsigned long, const unsigned short &) --> class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >", pybind11::arg("__position"), pybind11::arg("__n"), pybind11::arg("__x"));
		cl.def("erase", (class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >)) &std::vector<unsigned short>::erase, "C++: std::vector<unsigned short>::erase(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >) --> class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >", pybind11::arg("__position"));
		cl.def("erase", (class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > > (std::vector<unsigned short>::*)(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >, class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >)) &std::vector<unsigned short>::erase, "C++: std::vector<unsigned short>::erase(class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >, class __gnu_cxx::__normal_iterator<const unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >) --> class __gnu_cxx::__normal_iterator<unsigned short *, class std::vector<unsigned short, class std::allocator<unsigned short> > >", pybind11::arg("__first"), pybind11::arg("__last"));
		cl.def("swap", (void (std::vector<unsigned short>::*)(class std::vector<unsigned short, class std::allocator<unsigned short> > &)) &std::vector<unsigned short>::swap, "C++: std::vector<unsigned short>::swap(class std::vector<unsigned short, class std::allocator<unsigned short> > &) --> void", pybind11::arg("__x"));
		cl.def("clear", (void (std::vector<unsigned short>::*)()) &std::vector<unsigned short>::clear, "C++: std::vector<unsigned short>::clear() --> void");
	}
}
