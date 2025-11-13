---
description: Explains how to process and transform data in a stream using RxJS transformation operators, from simple transformations such as map, scan, mergeMap, switchMap, and concatMap to asynchronous transformations, buffering, and windowing. Practical patterns that take advantage of TypeScript's type safety will be introduced with abundant code examples.
---

# Transformation Operators

Transformation operators are used to transform and process data within the RxJS pipeline.
By transforming values into new forms, they allow for more flexible and powerful control over reactive data flow.


## List of Operators
### ◾ Simple Value Transformations

|Operator|Description|
|---|---|
|[map](./map)|Apply a transformation function to each value|

### ◾ Accumulation

|Operator|Description|
|---|---|
|[scan](./scan)|Cumulatively generate values|
|[reduce](./reduce)|Output only the final accumulated result|

### ◾ Pair and Grouping

|Operator|Description|
|---|---|
|[pairwise](./pairwise)|Process two consecutive values in pairs|
|[groupBy](./groupBy)|Group values based on a key|

### ◾ Asynchronous Transformation

|Operator|Description|
|---|---|
|[mergeMap](./mergeMap) |Transform each value into an Observable and merge in parallel|
|[switchMap](./switchMap) |Switch to the latest Observable|
|[concatMap](./concatMap) |Execute each Observable sequentially|
|[exhaustMap](./exhaustMap) |Ignore new inputs while executing|
|[expand](./expand) |Recursively expand results|

### ◾ Batch Processing

|Operator|Description|
|---|---|
|[buffer](./buffer) |Batch values at the timing of another Observable|
|[bufferTime](./bufferTime) |Batch values at regular intervals|
|[bufferCount](./bufferCount) |Batch values by specified count|
|[bufferWhen](./bufferWhen) |Buffering with dynamically controlled end conditions|
|[bufferToggle](./bufferToggle) |Buffering with independent control of start and end|
|[windowTime](./windowTime) |Split into sub-Observables at regular intervals|


## Practical Transformation Patterns

In real-world applications, the following processing is possible by combining transformation operators:

- Input validation and feedback
- Optimal control of asynchronous API requests
- Data shaping, aggregation, and normalization
- Batch processing and grouping of event streams

👉 For more information: [Practical Transformation Patterns](./practical-use-cases)

## 🚨 Notes

To avoid common mistakes when using transformation operators, see also:

- **[Side effects in map](/en/guide/anti-patterns/common-mistakes#5-side-effects-in-map)** - Use `map` as a pure function
- **[Inappropriate operator selection](/en/guide/anti-patterns/common-mistakes#12-inappropriate-operator-selection)** - Proper use of higher-order operators
