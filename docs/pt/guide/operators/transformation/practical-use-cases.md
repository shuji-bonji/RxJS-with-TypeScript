---
description: This section covers practical use cases of RxJS transformation operators (map, mergeMap, switchMap, concatMap, scan, buffer, etc.). Practical patterns for flexibly processing and transforming data such as user input processing, API response formatting, nested requests, data aggregation, stream splitting and batch processing are introduced with TypeScript code examples. You will learn transformation patterns frequently used in practice.
---

# Practical Transformation Patterns

Transformation operators are one of the most frequently used groups of operators in RxJS.
They play an essential role in reactive programming to flexibly process and transform data.

In this section, we will organize the patterns of utilization of the transformation operators by presenting typical practical examples.

## 💬 Typical Usage Patterns

| Pattern | Representative Operators | Description |
|:---|:---|:---|
| Simple value conversion | `map` | Apply conversion function to each value |
| Accumulation and aggregation | `scan`, `reduce` | Sequential accumulation of values |
| Nested asynchronous processing | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Generate and combine Observables |
| Batch processing and grouping | `bufferTime`, `bufferCount`, `windowTime` | Collective processing and partitioning management |
| Property extraction | `pluck` | Extract specific fields from objects |

## User Input Validation and Conversion

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

// Input field
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// Email validation function
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// Input processing
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'Please enter an email address',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: 'Please enter a valid email address',
          value: email,
        };
      }

      return {
        isValid: true,
        message: 'Email address is valid',
        value: email,
      };
    })
  )
  .subscribe((result) => {
    if (result.isValid) {
      emailStatus.textContent = '✓ ' + result.message;
      emailStatus.className = 'valid';
    } else {
      emailStatus.textContent = '✗ ' + result.message;
      emailStatus.className = 'invalid';
    }
  });
```

## Object Array Conversion and Aggregation

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

// Sales data
const sales = [
  { product: 'Laptop', price: 120000, quantity: 3 },
  { product: 'Tablet', price: 45000, quantity: 7 },
  { product: 'Smartphone', price: 85000, quantity: 4 },
  { product: 'Mouse', price: 3500, quantity: 12 },
  { product: 'Keyboard', price: 6500, quantity: 8 },
];

// Data conversion and aggregation
from(sales)
  .pipe(
    // Calculate total amount for each product
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // Add tax-included price
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // Convert back to array
    toArray(),
    // Calculate grand total
    map((items) => {
      const grandTotal = items.reduce((sum, item) => sum + item.total, 0);
      const grandTotalWithTax = items.reduce(
        (sum, item) => sum + item.totalWithTax,
        0
      );
      return {
        items,
        grandTotal,
        grandTotalWithTax,
      };
    })
  )
  .subscribe((result) => {
    console.log('Product details:', result.items);
    console.log('Grand total (excluding tax):', result.grandTotal);
    console.log('Grand total (including tax):', result.grandTotalWithTax);
  });
// Output:
// Product details: (5) [{…}, {…}, {…}, {…}, {…}]
// Grand total (excluding tax): 1109000
// Grand total (including tax): 1219900
```

## JSON Data Normalization

```ts
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs';

const resultBox = document.createElement('div');
resultBox.id = 'normalized-results';
document.body.appendChild(resultBox);

ajax
  .getJSON<any[]>('https://jsonplaceholder.typicode.com/users')
  .pipe(
    map((users) => {
      // Convert to object with ID as key
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // Flatten nested objects
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // Remove unnecessary nesting
          company: undefined,
          address: undefined,
        };
        userIds.push(user.id);
      });

      return {
        entities: normalizedUsers,
        ids: userIds,
      };
    })
  )
  .subscribe((result) => {
    const title = document.createElement('h3');
    title.textContent = 'Normalized User Data';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      Username: @${user.username}<br>
      Email: ${user.email}<br>
      Company: ${user.companyName}<br>
      Address: ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // Quick access to user by specific ID
    console.log('User ID 3:', result.entities[3]);
  });

```

## Combining Multiple Transformations

In real-world applications, it is common to use a combination of multiple transformation operators.

```ts
import { fromEvent, timer } from 'rxjs';
import {
  switchMap,
  map,
  tap,
  debounceTime,
  takeUntil,
  distinctUntilChanged,
} from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
  company: {
    name: string;
  };
};

// Search input
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// Search processing
fromEvent(searchInput, 'input')
  .pipe(
    // Get input value
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Wait 300ms
    debounceTime(300),
    // Ignore same value
    distinctUntilChanged(),
    // Show loading
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // API request (cancel previous request)
    switchMap((term) => {
      // Empty input returns no results
      if (term === '') {
        return [];
      }

      // Timeout handling (5 seconds)
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('API response timed out')),
        map(() => [{ error: 'Timeout' }])
      );

      // API call
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // Process results
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // Complete before timeout
          takeUntil(timeout$)
        );

      return response$;
    }),
    // Hide loading
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">No users found</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>Company: ${user.company}</p>
          </div>
        `
          )
          .join('');
      }
    } else {
      resultsContainer.innerHTML = `<div class="error">⚠️ ${result}</div>`;
    }
  });

```

## 🧠 Summary

- For simple conversions, use `map`
- For handling asynchronous operations, use `mergeMap`, `switchMap`, `concatMap`, `exhaustMap`
- For batch processing, use `bufferTime`, `bufferCount`
- For property extraction, use `pluck`
- **In actual apps, combining these operators is the norm**

Once you master the transformation operators, you can handle complex asynchronous data flows
intuitively and declaratively!
