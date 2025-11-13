---
description: The map operator is a basic conversion method that applies a function to each value in an Observable to generate a new value, and is often used for form formatting and API response processing.
---

# map - Apply a Conversion Function to Each Value

The `map` operator applies a specified function to **each value** in the stream to produce a new value after conversion.
Similar to the `Array.prototype.map` method of an array, but it works **on asynchronous streams**.


## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Output: 10, 20, 30
```

Applies the function `value => value * 10` to each value to produce a new value.

[🌐 RxJS Official Documentation - map](https://rxjs.dev/api/index/function/map)


## 💡 Typical Usage Patterns
- API response conversion (extract only necessary properties)
- Formatting of form input data
- Processing numbers and strings in streams
- Extract only necessary data from UI events


## 🧠 Practical Code Example (with UI)

This is an example of displaying an input numerical value multiplied by 2 in real time.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Create input field
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Enter a number';
document.body.appendChild(input);

// Create output field
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Input event stream
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Doubled value: ${result}`;
});
```

- The input value is doubled in real time and output.
- A simple data conversion chain is realized by continuously applying map.
