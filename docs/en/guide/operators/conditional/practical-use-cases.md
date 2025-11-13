---
description: Practical use cases of RxJS conditional operators (iif, defer) will be explained, including API fallback processing, cache strategies, dynamic data source selection, and conditional lazy evaluation. Specific patterns of use in situations requiring dynamic processing branches are introduced with TypeScript code examples. You will learn implementation patterns that can be immediately applied to actual application development.
---

# Practical Use Cases

RxJS conditional operators can be utilized to branch and switch streams according to dynamic states.
In this chapter, you can experience the utilization patterns of each operator through actual working code with UI.

## Selection of Different Data Sources Based on Conditions

```ts
import { iif, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs';

// Create UI
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>Data Source Selection App:</h3>';
document.body.appendChild(appContainer);

// Option selection
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// Checkbox (offline mode)
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'Offline Mode';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// Search ID input
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.value = '1';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// Search button
const searchButton = document.createElement('button');
searchButton.textContent = 'Search';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// Results area
const resultsArea = document.createElement('div');
resultsArea.style.padding = '15px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.borderRadius = '5px';
resultsArea.style.backgroundColor = '#f9f9f9';
resultsArea.style.minHeight = '150px';
appContainer.appendChild(resultsArea);

type User = {
  lastUpdated?: Date;
  fromCache?: boolean;
  id: number;
  name: string;
  email: string;
};
type ErrorResult = {
  error: boolean;
  message: string;
};

// Offline data (cache)
const cachedData: Record<number, User> = {
  1: { id: 1, name: 'Taro Yamada', email: 'yamada@example.com' },
  2: { id: 2, name: 'Hanako Sato', email: 'sato@example.com' },
  3: { id: 3, name: 'Ichiro Suzuki', email: 'suzuki@example.com' },
};

// Get data from online API (JSONPlaceholder)
function fetchUserFromApi(id: number) {
  console.log(`Getting user ID ${id} from API...`);

  // Actual API endpoint
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP error: ${response.status}`);
        }
        return response.json();
      })
    ),
    tap(() => console.log('API call succeeded')),
    catchError((err) => {
      console.error('API call failed:', err);
      throw new Error('API request failed');
    })
  );
}

// Get user from cache
function getUserFromCache(id: number) {
  console.log(`Getting user ID ${id} from cache...`);

  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => {
        throw new Error('User not found in cache');
      })
    )
  );
}

// Search button click
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;

  // Input validation
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML =
      '<p style="color: red;">Please enter a valid ID (1-10)</p>';
    return;
  }

  // Loading display
  resultsArea.innerHTML = '<p>Fetching data...</p>';

  // Select data source based on offline mode
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError((err) => {
        console.error('Cache error:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // Retry up to 2 times
      catchError((err) => {
        console.error('API error:', err);

        // Use cache as fallback if API fails
        return getUserFromCache(id).pipe(
          catchError(() =>
            of({ error: 'Both online API and cache failed' })
          )
        );
      })
    )
  ).subscribe({
    next: (result: any) => {
      if ('error' in result) {
        resultsArea.innerHTML = `<p style="color: red;">Error: ${result.message}</p>`;
      } else {
        const source = result.fromCache
          ? '<span style="color: orange;">(from cache)</span>'
          : '<span style="color: green;">(from API)</span>';

        resultsArea.innerHTML = `
          <h4>User Information ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>Name:</strong> ${result.name}</p>
          <p><strong>Email:</strong> ${result.email}</p>
          ${
            result.lastUpdated
              ? `<p><small>Last updated: ${new Date(
                  result.lastUpdated
                ).toLocaleString()}</small></p>`
              : ''
          }
        `;
      }
    },
    error: (err) => {
      resultsArea.innerHTML = `<p style="color: red;">Error: ${err.message}</p>`;
    },
  });
});

// Initial message
resultsArea.innerHTML = '<p>Click the button to fetch data</p>';


```



## Run-Time Branching and Fallback Strategies

In this example using `iif`, the data source is dynamically switched from "offline cache" and "online API" according to user operations and states.
Also, by combining `catchError` and `retry`, retries and fallback destinations can be defined in case of failure.

It is especially suited for the following use cases:

- Offline support in unstable network environments
- Cache utilization and online request switching
- Automatic retry and switching to alternate routes in case of API failure

## Performance Optimization Pattern

In more complex scenarios, optimized data acquisition patterns can be implemented by combining conditional operators.

```ts
import { fromEvent, Observable, of, throwError, timer } from 'rxjs';
import {
  switchMap,
  catchError,
  map,
  tap,
  debounceTime,
  distinctUntilChanged,
  withLatestFrom,
  delay,
  startWith,
} from 'rxjs';

// Create UI elements
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>Advanced Conditional Data Fetching:</h3>';
document.body.appendChild(optimizationContainer);

// Search UI
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Enter user ID (1-10)';
searchInput.value = '1';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = 'Search';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// Option settings
const optionsGroup = document.createElement('div');
optionsGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(optionsGroup);

const cacheCheckbox = document.createElement('input');
cacheCheckbox.type = 'checkbox';
cacheCheckbox.id = 'useCache';
cacheCheckbox.checked = true;
optionsGroup.appendChild(cacheCheckbox);

const cacheLabel = document.createElement('label');
cacheLabel.htmlFor = 'useCache';
cacheLabel.textContent = 'Use Cache';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = 'Force Refresh';
optionsGroup.appendChild(forceLabel);

// Results display area
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// Cache management
const cache = new Map<string, { data: any; timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30 seconds

// Get user data from actual API (JSONPlaceholder)
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // Invalid ID
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(
      () => new Error('Invalid user ID: please enter a number between 1 and 10')
    );
  }

  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();

  // Cache check (within expiry and not force refresh)
  if (
    !forceRefresh &&
    cachedItem &&
    now - cachedItem.timestamp < CACHE_EXPIRY
  ) {
    console.log(`Retrieved from cache: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true,
    }).pipe(delay(100)); // Simulate fast response
  }

  // Actual API request (JSONPlaceholder)
  console.log(`Fetching data from API: ${id}`);
  const apiUrl = `https://jsonplaceholder.typicode.com/users/${id}`;

  return of(null).pipe(
    switchMap(() =>
      fetch(apiUrl).then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP error: ${response.status}`);
        }
        return response.json();
      })
    ),
    map((userData) => {
      const processedData = {
        id: userData.id,
        name: userData.name,
        email: userData.email,
        lastUpdated: now,
        fromCache: false,
      };

      // Save to cache
      cache.set(cacheKey, {
        data: processedData,
        timestamp: now,
      });

      return processedData;
    }),
    catchError((err) => {
      console.error('API error:', err);
      throw new Error('API request failed');
    })
  );
}

// Monitor search condition changes
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map((event) => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// Monitor cache setting changes
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Monitor force refresh changes
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// Search button click event
const searchClick$ = fromEvent(searchButton, 'click');

// Execute search
searchClick$
  .pipe(
    // Get current input value, cache setting, force refresh setting
    withLatestFrom(
      searchTerm$,
      useCache$,
      forceRefresh$,
      (_, term, useCache, forceRefresh) => ({
        term,
        useCache,
        forceRefresh,
      })
    ),
    tap(() => {
      // Display search start
      optimizedResults.innerHTML = '<p>Searching...</p>';
    }),
    // Conditional stream using iif()
    switchMap(({ term, useCache, forceRefresh }) => {
      // If search term is empty
      if (!term) {
        return of({ error: 'Please enter a search term' });
      }

      // If cache is disabled
      if (!useCache) {
        return fetchUserData(term, true);
      }

      // Normal search (use cache & force refresh if needed)
      return fetchUserData(term, forceRefresh);
    }),
    // Error handling
    catchError((err) => {
      return of({ error: err.message });
    })
  )
  .subscribe({
    next: (result) => {
      if ('error' in result) {
        // Error display
        optimizedResults.innerHTML = `
        <p style="color: red;">Error: ${result.error}</p>
      `;
      } else {
        // Data display
        const source = result.fromCache
          ? '<span style="color: orange;">(from cache)</span>'
          : '<span style="color: green;">(from API)</span>';

        optimizedResults.innerHTML = `
        <h4>User Information ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>Name:</strong> ${result.name}</p>
        <p><strong>Email:</strong> ${result.email}</p>
        ${
          result.lastUpdated
            ? `<p><small>Last updated: ${new Date(
                result.lastUpdated
              ).toLocaleString()}</small></p>`
            : ''
        }
      `;
      }
    },
  });

// Initial message
optimizedResults.innerHTML =
  '<p>Enter a user ID and click the search button</p>';

```


---

## Operator Selection Guide

Many condition operators look similar and can be confusing, but each has a clear application purpose.
Below is a comparison of typical decision flows and characteristics.

## How to Choose a Condition Operator

| Operator | Use Case | Features |
|------------|------------|------|
| `iif` | Select one stream at runtime | Select one of two choices based on a condition |
| `partition` | Separate a stream into two streams based on a condition | Split the original stream into True/False based on a condition |
| `throwIfEmpty` | Detect empty streams | Throw an error if none of the values are issued |
| `defaultIfEmpty` | Use default value if empty | Provide fallback value if stream is empty |

### Selection Decision Flow

1. **Are there two options?**
   - Yes → Use `iif`
   - No → Next

2. **Do you want to split the stream?**
   - Yes → Use `partition`
   - No → Next

3. **Do you want to deal with empty streams?**
   - Yes → Do you want to treat empty streams as errors?
     - Yes → `throwIfEmpty`
     - No → `defaultIfEmpty`
   - No → Next

4. **Do you want to simply filter values based on a condition?**
   - Yes → Use `filter` operator (basic filtering operator)
   - No → Reexamine the purpose

## Summary

Conditional operators are powerful tools for controlling the flow of streams and branching processing based on specific conditions. The main points are as follows:

1. **Decision-based reactive flow**: Condition operators can be used to dynamically change processing based on events or data conditions.
2. **Enhanced error handling**: Condition operators can serve as an important part of your error handling strategy, enabling graceful handling of exception cases.
3. **Optimization opportunities**: Conditional execution avoids unnecessary processing and optimizes costly operations, especially network requests and hardware access.
4. **Complex application flows**: Complex business logic and state management can be expressed declaratively by combining multiple conditional operators.

Condition operators are especially valuable when implementing error handling, caching strategies, fallback mechanisms, and conditional execution patterns using RxJS. Combined with other operators, they allow you to build complex application flows in a declarative and type-safe manner.
