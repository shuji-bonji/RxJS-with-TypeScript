---
description: The distinctUntilKeyChanged operator focuses on a specific property within an object stream and outputs only when that value differs from the previous one. It efficiently skips consecutive duplicate data and is useful for detecting state changes and optimizing list updates.
titleTemplate: ':title'
---

# distinctUntilKeyChanged - Detect Key Changes

The `distinctUntilKeyChanged` operator focuses on a specific key (property) of an object and outputs only when that value differs from the previous one.
It is useful for efficiently skipping consecutive duplicates.


## 🔰 Basic Syntax and Usage

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Tanaka' },
  { id: 2, name: 'Tanaka' }, // Same name, skip
  { id: 3, name: 'Sato' },
  { id: 4, name: 'Suzuki' },
  { id: 5, name: 'Suzuki' }, // Same name, skip
  { id: 6, name: 'Tanaka' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Output:
// { id: 1, name: 'Tanaka' }
// { id: 3, name: 'Sato' }
// { id: 4, name: 'Suzuki' }
// { id: 6, name: 'Tanaka' }
```

- Outputs only when the value of the specified property `name` changes.
- Other properties (e.g. `id`) are not compared.

[🌐 RxJS Official Documentation - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Typical Usage Patterns

- Update list display only when a specific property changes
- Detect only changes in specific attributes in event streams
- Control duplicate removal on a key basis


## 🧠 Practical Code Example (with UI)

Enter a name in the text box and press Enter to register it.
**If the same name is entered consecutively, it is ignored**, and it is added to the list only when a different name is entered.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Create output area
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'distinctUntilKeyChanged Practical Example';
output.appendChild(title);

// Input form
const input = document.createElement('input');
input.placeholder = 'Enter name and press Enter';
document.body.appendChild(input);

// Input event stream
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `User input: ID=${user.id}, Name=${user.name}`;
  output.appendChild(item);
});
```

- If the same name is entered consecutively, it is skipped.
- It is displayed only when a new name is entered.
