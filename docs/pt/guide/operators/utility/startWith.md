---
description: The startWith operator inserts the specified initial value before the Observable emits the value, and is suitable for state initialization and initial display of the UI.
---

# startWith - Provide Initial Value

The `startWith` operator is an operator to **emit the specified initial value before the source Observable emits the value**.
It is utilized for state management, initial display, placeholder values, etc.


## 🔰 Basic Syntax and Operation

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// Output:
// A
// B
// C
```

Thus, `startWith` adds `'A'` first, followed by the values of the source Observable.

[🌐 RxJS Official Documentation - startWith](https://rxjs.dev/api/index/function/startWith)

## 💡 Typical Usage Example

This is useful when you want to set initial values for states or counters. Here is an example of a counter that starts with an initial value of `100`.

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

interval(1000)
  .pipe(
    startWith(-1), // Insert -1 first
    scan((acc, curr) => acc + 1, 100), // Increment from initial value 100
    take(10) // Output 10 times total
  )
  .subscribe(console.log);
// Output:
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 Practical Code Example (with UI)

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

// Output display area
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>startWith Example:</h3>';
document.body.appendChild(startWithOutput);

// Counter display area
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// Values list display area
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// Counter stream (every 1 second)
interval(1000)
  .pipe(
    // Start with 100 first
    startWith(-1),
    // Add 1 to each value to the previous value
    scan((acc, curr) => acc + 1, 100),
    // End after 10 times
    take(10)
  )
  .subscribe((count) => {
    // Update counter display
    counterDisplay.textContent = count.toString();

    // Add value to list
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `Initial value: ${count} (added with startWith)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `Next value: ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ Summary

- `startWith` is useful for situations where you want to **insert a fixed value first**
- Commonly used for state initialization, UI placeholders, initial display of forms, etc.
- Used in combination with `scan`, `combineLatest`, etc. to **build the foundation for state management**
