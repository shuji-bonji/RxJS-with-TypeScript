---
description: RxJS's filtering operators are used to extract only the necessary data from the stream based on conditions and time, contributing to improved performance.
---

# Filtering Operators

RxJS's filtering operators are important tools for selecting only the necessary data from a stream and not letting unnecessary data through.
This greatly improves the efficiency and performance of the application.

Filtering operators are a set of RxJS operators that sort the values in a stream, allowing only those that meet certain criteria to pass through.
By controlling the flow of data and processing only the values you need, you can build an efficient data processing pipeline.


## List of Operators
### ◾ Basic Filtering Operators

| Operator | Description |
|:---|:---|
| [filter](./filter) | Only let through values that match the condition |
| [take](./take) | Get only the specified number of first values |
| [takeLast](./takeLast) | Get the specified number of last values |
| [takeWhile](./takeWhile) | Get values while the condition is met |
| [skip](./skip) | Skip the specified number of first values |
| [skipLast](./skipLast) | Skip the specified number of last values |
| [skipWhile](./skipWhile) | Skip values while the condition is met |
| [skipUntil](./skipUntil) | Skip values until another Observable fires |
| [first](./first) | Get the first value, or the first value that meets a condition |
| [last](./last) | Get the last value, or the last value that meets a condition |
| [elementAt](./elementAt) | Get the value at a specified index |
| [find](./find) | Find the first value that meets a condition |
| [findIndex](./findIndex) | Get the index of the first value that meets a condition |
| [ignoreElements](./ignoreElements) | Ignore all values and only pass through completions/errors |


### ◾ Time-based Filtering Operators

| Operator | Description |
|:---|:---|
| [debounceTime](./debounceTime) | Issue the last value if no input is received for a specified time |
| [throttleTime](./throttleTime) | Pass through the first value and ignore new values for the specified time |
| [auditTime](./auditTime) | Issue the last value after a specified time |
| [audit](./audit) | Control the period with a custom Observable and issue the last value |
| [sampleTime](./sampleTime) | Sample the latest value at specified time intervals |


### ◾ Condition-based Filtering Operators

| Operator | Description |
|:---|:---|
| [distinct](./distinct) | Remove all duplicate values (output only unique values) |
| [distinctUntilChanged](./distinctUntilChanged) | Remove consecutive duplicate values |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Detect only changes in specific properties |


## Practical Use Cases

- [Practical Use Cases](./practical-use-cases.md) presents practical examples of combining multiple filtering operators (real-time search, infinite scrolling, etc.).
