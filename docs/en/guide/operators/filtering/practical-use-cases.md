---
description: Practical use cases for RxJS filtering operators (debounceTime, throttleTime, distinctUntilChanged, filter, etc.) are explained. Learn practical patterns to extract only the values you need from streams, such as real-time search, infinite scrolling, controlling high-frequency events, deduplication, etc., with TypeScript code examples. You will learn useful implementation techniques for UI event handling and performance optimization.
---

# Practical Use Cases

## Real-Time Search Filtering of User Input

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// Build UI
const searchInput = document.createElement('input');
searchInput.placeholder = 'Enter search term (3+ characters)';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// Event stream
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `Starting search for "${searchTerm}"...`;
  });

```

- **Processes only confirmed input** at 300ms intervals.
- **Searches are performed only when 3 or more characters** are entered.
- **Consecutive entries of the same word** are ignored.


## Infinite Scrolling Simulation

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// Build UI
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// Add initial data
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `Item ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// Scroll event stream
fromEvent(scrollArea, 'scroll')
  .pipe(
    throttleTime(200),
    map(() => ({
      scrollTop: scrollArea.scrollTop,
      scrollHeight: scrollArea.scrollHeight,
      clientHeight: scrollArea.clientHeight,
    })),
    map(
      ({ scrollTop, scrollHeight, clientHeight }) =>
        (scrollTop + clientHeight) / scrollHeight
    ),
    distinctUntilChanged(),
    filter((ratio) => ratio > 0.8),
    scan((page) => page + 1, 1),
    filter((page) => page <= 5)
  )
  .subscribe((page) => {
    addItems(page);
  });

```

- When the scroll position reaches **80% or more**, the next items are loaded.
- **Auto-loads up to 5 pages**.
- **Scroll events** are throttled **every 200ms**.


## Summary of How to Choose Filtering Operators

| What You Want To Do | Operator | Description |
|:---|:---|:---|
| Only pass data matching condition | `filter` | Most basic filtering |
| Get only first few items | `take`, `first` | Limit number of items acquired |
| Wait until input is confirmed | `debounceTime` | Ideal for form input |
| Process only at fixed intervals | `throttleTime` | Apply to scroll, resize, etc. |
| Ignore consecutive same values | `distinctUntilChanged` | Prevent wasteful re-processing of identical data |


## Summary

- Filtering operators are essential for controlling data streams.
- They are not only powerful when used alone, but even more so when **combined**.
- Directly lead to **improved efficiency and performance** in event-driven applications and UI development.
