---
description: mergeScan is an RxJS conversion operator that performs asynchronous accumulation processing, combining scan and mergeMap behavior. It is best suited for situations where asynchronous accumulation is required, such as cumulative aggregation of API responses, executing the next request based on previous results, and cumulative data acquisition across multiple pages during pagination. The number of concurrent executions can be controlled with the concurrent parameter.
---

# mergeScan - Accumulation with Asynchronous Processing

The `mergeScan` operator performs an **asynchronous accumulation** of each value in the stream.
It works like a combination of `scan` and `mergeMap`, keeping the accumulated values, converting each value to a new Observable, and using the result for the next accumulation operation.

## 🔰 Basic Syntax and Usage

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take } from 'rxjs';

interval(1000).pipe(
  take(5),
  mergeScan((acc, curr) => {
    // Asynchronous processing for each value (returned immediately here)
    return of(acc + curr);
  }, 0)
).subscribe(console.log);

// Output: 0, 1, 3, 6, 10
```

- `acc` is the cumulative value, `curr` is the current value.
- The cumulative function must **return an Observable**.
- The result of processing each value is accumulated.

[🌐 RxJS Official Documentation - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 Typical Usage Patterns

- Accumulate and aggregate API responses
- Execute next API request based on previous results
- Asynchronous cumulative processing of real-time data
- Cumulative acquisition of data from multiple pages with pagination

## 📊 Difference from scan

| Operator | Cumulative Function Return Value | Use Case |
|--------------|------------------|--------------|
| `scan` | Return value directly | Synchronous accumulation process |
| `mergeScan` | Return Observable | Asynchronous accumulation process |

```ts
// scan - Synchronous processing
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)

// mergeScan - Asynchronous processing
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr).pipe(delay(100)), 0)
)
```

## 🧠 Practical Code Example (API Cumulative Acquisition)

This is an example where new data is added to the previous result each time a button is clicked.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeScan, delay, take, map } from 'rxjs';

// Create button
const button = document.createElement('button');
button.textContent = 'Fetch Data';
document.body.appendChild(button);

// Create output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Dummy API (returns data with delay)
const fetchData = (page: number) => {
  return of(`Data ${page}`).pipe(delay(500));
};

// Cumulative acquisition on click event
fromEvent(button, 'click').pipe(
  take(5), // Maximum 5 times
  mergeScan((accumulated, _, index) => {
    const page = index + 1;
    console.log(`Fetching page ${page}...`);

    // Add new data to accumulated data
    return fetchData(page).pipe(
      map(newData => [...accumulated, newData])
    );
  }, [] as string[])
).subscribe((allData) => {
  output.innerHTML = `
    <div>Fetched data:</div>
    <ul>${allData.map(d => `<li>${d}</li>`).join('')}</ul>
  `;
});
```

- Data is retrieved asynchronously with each click.
- New data is added to the previous result (`accumulated`).
- **Accumulated results are updated in real time**.

## 🎯 Practical Example: Accumulation with Concurrency Control

The `mergeScan` has a `concurrent` parameter to control the number of concurrent executions.

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take, delay } from 'rxjs';

interface RequestLog {
  total: number;
  logs: string[];
}

interval(200).pipe(
  take(10),
  mergeScan((acc, curr) => {
    const timestamp = new Date().toLocaleTimeString();
    console.log(`Request ${curr} started: ${timestamp}`);

    // Each request takes 1 second
    return of({
      total: acc.total + 1,
      logs: [...acc.logs, `Request ${curr} completed: ${timestamp}`]
    }).pipe(delay(1000));
  }, { total: 0, logs: [] } as RequestLog, 2) // Concurrency: 2
).subscribe((result) => {
  console.log(`Total: ${result.total} items`);
  console.log(result.logs[result.logs.length - 1]);
});
```

- With `concurrent: 2`, up to two requests are executed simultaneously.
- The third and subsequent requests will wait until the previous request completes.

## ⚠️ Notes

### 1. Error Handling

If an error occurs within the accumulation function, the entire stream will stop.

```ts
source$.pipe(
  mergeScan((acc, curr) => {
    return apiCall(curr).pipe(
      map(result => acc + result),
      catchError(err => {
        console.error('Error occurred:', err);
        // Continue with accumulated value maintained
        return of(acc);
      })
    );
  }, 0)
)
```

### 2. Memory Management

Be careful not to let the cumulative value become too large.

```ts
// Bad example: Unlimited accumulation
mergeScan((acc, curr) => of([...acc, curr]), [])

// Good example: Keep only the latest N items
mergeScan((acc, curr) => {
  const newAcc = [...acc, curr];
  return of(newAcc.slice(-100)); // Keep only the latest 100 items
}, [])
```

### 3. Use scan for Synchronous Processing

If you do not need asynchronous processing, use simple `scan`.

```ts
// mergeScan is unnecessary
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr), 0)
)

// scan is sufficient
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)
```

## 🔗 Related Operators

- [`scan`](/en/guide/operators/transformation/scan) - Synchronous cumulative processing
- [`reduce`](/en/guide/operators/transformation/reduce) - Outputs final cumulative value only on completion
- [`mergeMap`](/en/guide/operators/transformation/mergeMap) - Asynchronous mapping (no accumulation)
- [`expand`](/en/guide/operators/transformation/expand) - Recursive expansion process
