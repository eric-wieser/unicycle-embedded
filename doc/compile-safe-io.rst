Compiler-verification of pin assignments
========================================

This project uses a novel approach to selecting microprocessor peripherals from
their pin numbers.

A typical microprocessor has multiple different features available on each pin.
However, very rarely is every feature available on every pin, and the mapping
between these features and their pins is only discoverable through a very long
table in the data sheet. For instance, we might find that ``OC1`` is available
on pin ``3``.

The result is typically that the source code does not contain any reference to
pin ``3`` at all, and only refers to the underlying hardare, ``OC1``. This is
bad for maintainability, as it tells the programmer nothing about how to check
the wiring, or what to change should the wiring change.

Obviously then, we want a function to convert pin numbers into their
corresponding hardware names, and this is not a hard task. But this creates
a new danger - it encourages the programmer to change pin a pin to one that
no longer supports the features needed! This kind of mistake can be caught at
run-time, but when programming takes a few minutes, or getting feedback is
difficult, this is not acceptable.

C++11 introduces a new keyword called `constexpr`. When attached to a variable,
this tells the compiler that its value must be calculated at compile-time. The
trick then, is to produce a function that for a valid pin, *is* calculable at
compilation-time, and for an invalid pin, is *not* calculable at
compilation-time. We capitalize on the fact that the compiler only cares about
the compile-time-calculability of the code-path it takes::

    // note: no constexpr
    int failure(const char* msg) { report_error(msg); while(1); }

    constexpr int must_be_even(int i) {
        return i % 2 == 0
            ? i                     // calculable at compile-time
            : failure("Not even");  // not calculable at compile-time
    }

This almost works as intended::

    constexpr int ok            = must_be_even(2);
    constexpr int compile_error = must_be_even(3);
    int           ok            = must_be_even(2);
    int           runtime_error = must_be_even(3);

There is a isocpp paper [N3583]_ and a `follow-up`_ paper that details possible
language changes and workaround to the problem of this third line. The solution
opted for was to encourage writing these as::

    constexpr int ok            = must_be_even<2>();
    constexpr int compile_error = must_be_even<3>();
    int           compile_error = must_be_even<2>();
    int           compile_error = must_be_even<3>();



..  http://stackoverflow.com/a/20648199/102441

.. [N3583] Scott Schurr, Exploring constexpr at Runtime, 2013.
           https://isocpp.org/files/papers/n3583.pdf
.. _`follow-up`: https://drive.google.com/file/d/0B0-xyi_NILkZZEZQT2VPY21BVTQ/edit


API documentation
-----------------

.. doxygenfile:: io.h
