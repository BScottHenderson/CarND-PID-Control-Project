### Twiddle

The twiddle algorithm is quite simple and, in the generic case, can be used to determine values for any number of parameters. In our vehicle steering case we have just three parameters corresponding to the three gain constants required for our PID controller. The twiddle algorithm requires some way of running the desired process, i.e., the process to be controlled via the PID controller, and obtaining an error value, CTE in our case.

Assume that we have a function called _run_ that will take our current list of parameter values and return an error value, e.g., _error = run(p)_

The only other required input for the twiddle algorithm is a _tolerance_ value that determines when the algorithm stops.

#### Initialize

Initialize each parameter, _p_, to 0.0
Initialize an equivalent-length list of delta values, _dp_, to 1.0
Initialize the _best-error_ value: _best-error = run(p)_

#### Loop

If _sum(dp)_ <= _tolerance_ then we're done. Return the current list of parameter values, _p_
For each parameter index, _i_, in the list _p_:
    Replace _p[i]_ with _p[i] + dp[i]_
    _error_ = _run(p)_
    If _error_ < _best-error_:
        _best-error_ = _error_
        Adjust _dp[i]_ by multiplying by a 'success factor', e.g., 1.1
        The intent here is to make a larger change in the next iteration since we were successful in this iteration.
    Else
        Replace _p[i]_ with _p[i] - dp[i]_ (taking care that we use the original _p[i]_ value)
        _error_ = _run(p)_
        If _error_ < _best-error_:
            _best-error_ = _error_
            Multiply _dp[i]_ by the 'success factor'
        Else
            Restore the original value for _p[i]_
            Adjust _dp[i]_ by multiplying by a 'failure factor', e.g., 0.9
            The intent here is to make a smaller change in the next iteration since we were not successful in this iteration.

### Results

I attempted to implement the twiddle algorithm to determine values for the PID constants but was ultimately not successful.

I am not entirely sure why I was unable to make the twiddle algorithm work. The algorithm itself is simple enough but perhaps does not work as well with 2D data? The example used in the lectures was for a vehicle traveling in a straight line - no curves. The "real" problem, of course, has curves and the vehicle will stall or crash (either way effectively coming to permanent rest) if the CTE becomes too large. It seems likely that some other parameter estimation mechanism would work better in this case. Trial and error was interesting but quite time consuming and it is not at all clear that I ended up with an optimal solution.
