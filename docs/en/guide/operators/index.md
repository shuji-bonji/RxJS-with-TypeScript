---
description: RxJS operators are categorized into seven groups - transformation, filtering, combination, utility, conditional, error handling, and multicasting. Learn practical TypeScript usage with comprehensive operator lists and pipeline concepts.
---

# Understanding Operators

RxJS operators are a set of functions for transforming, composing, and controlling Observable data streams.

Operators are usually used in combination with several others, and this is where the "pipeline" comes in.
- [What is RxJS Pipeline](./pipeline.md)

In RxJS, operators fall into the following categories


## List of categories

- [Transformation Operators](./transformation/)
- [Filtering Operators](./filtering/)
- [Combination Operators](./combination/)
- [Utility Operators](./utility/)
- [Conditional Operators](./conditional/)
- [Error Handling Operators](../error-handling/strategies.md)
- [Multicasting Operators](./multicasting/)

Each category contains a number of useful operators.
See each category for details.


## List of Operators

For a detailed description of each operator, click on the link to browse.

<table style="overflow: visible;">
  <caption>
   List of Operator categories
  </caption>
  <thead>
    <tr>
      <th scope="col">Category</th>
      <th scope="col">Operator</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <!-- Transformation Operators -->
    <tr>
      <th scope="row" rowspan="15"><a href="./transformation/">Transformation</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>Converts each value</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>Accumulate values and output intermediate results</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>Accumulate all values and output only the final result</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>Processes two consecutive values in pairs</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>Grouping streams by key</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>Parallel execution of asynchronous processing</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>Execute only the latest asynchronous processing (cancel older processing)</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>Execute asynchronous processes sequentially</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>Ignore new processes during execution</td>
    </tr>
    <tr>
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>Recursively expand results</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>Publish values in an array</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>Publish values at specified time intervals</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>Publish values in batches of specified number of values</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>Buffering with dynamically controlled end conditions</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>Buffering with independent control of start and end</td>
    </tr>
    <!-- Filtering Operators -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">Filtering</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>Only let through values that match the condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>Get only the first N values</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>Get the last N values</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>Get values while the condition is met</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>Skip the first N values</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>Skip the last N values</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>Skip values while the condition is satisfied</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>Skip values until another Observable fires</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>Get the first value or the first value satisfying a condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>Get the last value or the last value that satisfies the condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>Get the value at a given index</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>Find the first value that satisfies a condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>Get the index of the first value that satisfies the condition</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>Issue the last value if no input is received for a specified time</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>Pass through the first value and ignore the new value for the specified time</td>
    </tr>
    <tr>
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>Issue last value after specified time</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>Issue last value with custom Observable to control period</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>Sample latest value at specified time interval</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>Ignore all values and only pass through completions/errors</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>Remove all duplicate values (output only unique values)</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>Remove consecutive duplicate values</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>Detect only changes in specific properties of an object</td>
    </tr>
    <!-- Combination Operators (Pipeable) -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">Combination (Pipeable)</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>Join other Observables in sequence after completion</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>Combine multiple Observables simultaneously</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>Combine the latest value of each Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>Pair values in corresponding order</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>Adopt only the first Observable that fires</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>Append other latest values to the main stream</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Flatten Higher-order Observables in parallel</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Flatten Higher-order Observable in sequence</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>Switch to the latest Higher-order Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>Ignore new Higher-order Observable during execution</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>Combines the latest values of all internal Observables</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>Pair the corresponding values of each internal Observable</td>
    </tr>
    <!-- Utility Operators -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">Utility</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>Perform side effects (e.g., log output)</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>Perform post-processing on completion or error</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>Delay all values for a specified time</td>
    </tr>
    <tr>
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>Delay each value dynamically with a separate Observable</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>Issue an error if a value does not arrive within a specified time</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>Retrieve values until another Observable issues a value</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>Retry up to specified number of times on error</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>Repeat a specified number of times after completion</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>Adds an initial value to the beginning of the stream</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>Publish all values together in an array</td>
    </tr>
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>Convert a notification to a Notification object</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Convert the Notification object back to a normal notification</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>Use the scheduler to control when values are published</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>Use the scheduler to control when to start subscribing</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>Add a timestamp to each value</td>
    </tr>
    <!-- Conditional Operators -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">Conditional</a></th>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>If no value is available, issue default value</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>Determine if all values satisfy the condition</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>Determines if no value was issued</td>
    </tr>
    <!-- Error Handling -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">Error Handling</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>Catch errors and perform fallback processing</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>Retry a specified number of times on error</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>Retry with custom conditions</td>
    </tr>
    <!-- Multicasting -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">Multicasting</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Share Observable among multiple subscribers</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>Cache the latest N values and replay them to new subscribers</td>
    </tr>
  </tbody>
</table>
