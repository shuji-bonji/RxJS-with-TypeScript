---
description: toArray is an RxJS utility operator that combines all values emitted until Observable completes into a single array. This is ideal for situations where you want to treat the entire stream as an array, such as batch processing, UI display after batch acquisition, and aggregate processing. Because it accumulates values until completion, it cannot be used with infinite streams.
---

# toArray - Convert Values to Array

The `toArray` operator is an operator that **combines all values emitted by Observable to completion into a single array**.
This is useful for batch processing, UI display after batch retrieval, aggregation, etc.


## 🔰 Basic Syntax and Operation

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Output:
// [1, 2, 3]
```

All values are combined into a single array, which is emitted upon Observable completion.

[🌐 RxJS Official Documentation - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Typical Usage Example

This can be used in situations where you want to process multiple asynchronous results at once or output them to the UI in a batch.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Receive all at completion:', result);
  });

// Output:
// Receive all at completion: [0, 1, 2, 3, 4]
```


## 🧪 Practical Code Example (with UI)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Output display area
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>toArray Example:</h3>';
document.body.appendChild(toArrayOutput);

// Individual values display area
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Individual Values:</h4>';
toArrayOutput.appendChild(individualValues);

// Array result display area
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Array Result:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// Subscribe to individual values
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Value: ${val}`;
    individualValues.appendChild(valueItem);
  });

// Subscribe to same stream as array
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Result array: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Display array elements individually
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Summary

- `toArray` **emits an array of all values at completion**
- Ideal for situations where you want to handle the entire stream in aggregate
- Combined with `concatMap`, `delay`, etc., it can be used for **asynchronous sequential batch processing**
