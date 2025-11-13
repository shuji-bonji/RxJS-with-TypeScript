---
description: Practical use cases for RxJS join operators (combineLatest, forkJoin, merge, concat, withLatestFrom, etc.) will be explained. Practical patterns of combining multiple Observables such as form input validation and API integration, parallel execution of multiple requests, real-time data synchronization, and sequential processing of streams will be presented with TypeScript code examples.
---

# Practical Use Cases

In this chapter, we will introduce **practical use cases** that take advantage of RxJS's combination operators.
Deepen your understanding through scenarios useful for actual application development, such as UI operations and API communication.

## Form Input Validation and API Requests

An example of validating multiple form inputs using `combineLatest`.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs';

// Create form UI
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>User Registration Form:</h3>';
document.body.appendChild(formContainer);

// Name input
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Name: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// Email input
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// Password input
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'Password: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// Submit button
const submitButton = document.createElement('button');
submitButton.textContent = 'Register';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// Validation message
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Name validation
const name$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? 'Name must be at least 2 characters' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Name must be at least 2 characters',
  })
);

// Email validation
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value)
        ? 'Please enter a valid email address'
        : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Please enter a valid email address',
  })
);

// Password validation
const password$ = fromEvent(passwordInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'Password must be at least 6 characters' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Password must be at least 6 characters',
  })
);

// Combine validation state of all fields
combineLatest([name$, email$, password$])
  .pipe(debounceTime(300))
  .subscribe(([nameState, emailState, passwordState]) => {
    // Check if form is valid
    const isFormValid =
      nameState.valid && emailState.valid && passwordState.valid;
    submitButton.disabled = !isFormValid;

    // Display error messages
    if (!isFormValid) {
      const errors = [
        nameState.error,
        emailState.error,
        passwordState.error,
      ].filter((error) => error !== null);

      validationMessage.textContent = errors.join('\n');
    } else {
      validationMessage.textContent = '';
    }
  });

// Submit button click event
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value,
  };

  // Display form data (in real use, send to API)
  const successMessage = document.createElement('div');
  successMessage.textContent = 'Registration completed!';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);

  console.log('Submitted data:', formData);
});

```

## Concurrent Requests and Loading State Management

Here is an example of using `forkJoin` to process multiple API requests in parallel and summarize the results.

```ts
import {
  forkJoin,
  of,
  throwError,
  Observable,
  ObservableInputTuple,
} from 'rxjs';
import { catchError, delay, finalize } from 'rxjs';

// Interface definitions
interface User {
  id: number;
  name: string;
  email: string;
}

interface Post {
  id: number;
  title: string;
  content: string;
}

interface WeatherSuccess {
  city: string;
  temp: number;
  condition: string;
}

interface WeatherError {
  error: string;
}

type Weather = WeatherSuccess | WeatherError;

// Result type definition
interface ApiResponse {
  user: User;
  posts: Post[];
  weather: Weather;
}

// Create UI elements
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Multiple API Requests Example:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'Load Data';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// Simulate API requests
function fetchUser(id: number): Observable<User> {
  // Successful request
  return of({
    id,
    name: `User${id}`,
    email: `user${id}@example.com`,
  }).pipe(
    delay(2000) // 2 second delay
  );
}

function fetchPosts(userId: number): Observable<Post[]> {
  // Successful request
  return of([
    { id: 1, title: `${userId}'s Post 1`, content: 'Content...' },
    { id: 2, title: `${userId}'s Post 2`, content: 'Content...' },
  ]).pipe(
    delay(1500) // 1.5 second delay
  );
}

function fetchWeather(city: string): Observable<WeatherSuccess> {
  // Sometimes fails
  const shouldFail = Math.random() > 0.7;

  if (shouldFail) {
    return throwError(() => new Error('Failed to fetch weather data')).pipe(
      delay(1000)
    );
  }

  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['Sunny', 'Cloudy', 'Rainy'][Math.floor(Math.random() * 3)],
  }).pipe(
    delay(1000) // 1 second delay
  );
}

// Execute multiple requests on button click
loadButton.addEventListener('click', () => {
  // Reset UI
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'Loading data...';
  loadButton.disabled = true;

  // Execute multiple API requests concurrently
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('Tokyo').pipe(
      // Error handling
      catchError((error: Error) => {
        console.error('Weather API error:', error);
        return of<WeatherError>({ error: error.message });
      })
    ),
  } as ObservableInputTuple<ApiResponse>)
    .pipe(
      // Cleanup on completion
      finalize(() => {
        loadingIndicator.style.display = 'none';
        loadButton.disabled = false;
      })
    )
    .subscribe((results: ApiResponse) => {
      // Display user info
      const userInfo = document.createElement('div');
      userInfo.innerHTML = `
      <h4>User Information</h4>
      <p>Name: ${results.user.name}</p>
      <p>Email: ${results.user.email}</p>
    `;
      userInfo.style.margin = '10px 0';
      userInfo.style.padding = '10px';
      userInfo.style.backgroundColor = '#f0f0f0';
      userInfo.style.borderRadius = '5px';
      resultContainer.appendChild(userInfo);

      // Display posts
      const postsInfo = document.createElement('div');
      postsInfo.innerHTML = `
      <h4>Posts (${results.posts.length})</h4>
      <ul>
        ${results.posts
          .map((post: { title: string }) => `<li>${post.title}</li>`)
          .join('')}
      </ul>
    `;
      postsInfo.style.margin = '10px 0';
      postsInfo.style.padding = '10px';
      postsInfo.style.backgroundColor = '#f0f0f0';
      postsInfo.style.borderRadius = '5px';
      resultContainer.appendChild(postsInfo);

      // Display weather info
      const weatherInfo = document.createElement('div');

      if ('error' in results.weather) {
        weatherInfo.innerHTML = `
        <h4>Weather Information</h4>
        <p style="color: red;">Error: ${results.weather.error}</p>
      `;
      } else {
        weatherInfo.innerHTML = `
        <h4>Weather Information</h4>
        <p>City: ${results.weather.city}</p>
        <p>Temperature: ${results.weather.temp}°C</p>
        <p>Condition: ${results.weather.condition}</p>
      `;
      }

      weatherInfo.style.margin = '10px 0';
      weatherInfo.style.padding = '10px';
      weatherInfo.style.backgroundColor = '#f0f0f0';
      weatherInfo.style.borderRadius = '5px';
      resultContainer.appendChild(weatherInfo);
    });
});

```

## Cancelable Search Function

Here is an example of combining `withLatestFrom` and `race` to implement a timeout or cancelable search function.

```ts
import { fromEvent, timer, race, of, EMPTY } from 'rxjs';
import {
  map,
  debounceTime,
  switchMap,
  tap,
  delay,
  catchError,
  takeUntil,
} from 'rxjs';

// Create search UI
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Cancelable Search:</h3>';
document.body.appendChild(searchContainer);

// Search input field
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Enter search term...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// Cancel button
const cancelButton = document.createElement('button');
cancelButton.textContent = 'Cancel';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// Search results area
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// Simulate search request
function searchApi(term: string) {
  console.log(`Starting search for "${term}"...`);

  // Simulate search results
  return of([
    `Search result 1 for "${term}"`,
    `Search result 2 for "${term}"`,
    `Search result 3 for "${term}"`,
  ]).pipe(
    // Random delay between 2-5 seconds
    delay(2000 + Math.random() * 3000),
    // Error handling
    catchError((err) => {
      console.error('Search error:', err);
      return EMPTY;
    })
  );
}

// Cancel event
const cancel$ = fromEvent(cancelButton, 'click');

// Search event
const search$ = fromEvent(searchInput, 'input')
  .pipe(
    // Get input value
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Wait 300ms
    debounceTime(300),
    // Ignore empty searches
    tap((term) => {
      if (term === '') {
        resultsContainer.innerHTML = '<p>Please enter a search term</p>';
      }
    }),
    // Don't process empty searches
    switchMap((term) => {
      if (term === '') {
        return EMPTY;
      }

      // Show searching message
      resultsContainer.innerHTML = '<p>Searching...</p>';

      // Timeout handling (5 seconds)
      const timeout$ = timer(5000).pipe(
        tap(() => console.log('Search timed out')),
        map(() => ({ type: 'timeout', results: null }))
      );

      // API request
      const request$ = searchApi(term).pipe(
        map((results) => ({ type: 'success', results })),
        // Cancel if cancel button is pressed
        takeUntil(
          cancel$.pipe(
            tap(() => {
              console.log('Search was cancelled');
              resultsContainer.innerHTML = '<p>Search was cancelled</p>';
            })
          )
        )
      );

      // Race between timeout and request completion
      return race(request$, timeout$);
    })
  )
  .subscribe((response) => {
    if (response.type === 'success') {
      // Search successful
      resultsContainer.innerHTML = '<h4>Search Results:</h4>';

      if (response.results?.length === 0) {
        resultsContainer.innerHTML += '<p>No results found</p>';
      } else {
        const list = document.createElement('ul');
        response.results?.forEach((result) => {
          const item = document.createElement('li');
          item.textContent = result;
          list.appendChild(item);
        });
        resultsContainer.appendChild(list);
      }
    } else if (response.type === 'timeout') {
      // Timeout
      resultsContainer.innerHTML =
        '<p style="color: red;">Search timed out. Please try again.</p>';
    }
  });

```


## Combination Operator Comparison and Selection Guide

Compare the differences between multiple combination operators and help you choose the right one for your use case.

| Operator | Timing | Output | Use Case |
|------------|------------|-----|------------|
| `merge` | Concurrent execution | Output in order of occurrence | Monitor multiple source events simultaneously |
| `concat` | Sequential execution | Output in order | Async tasks where order matters |
| `combineLatest` | Requires at least one value from all sources | Combination of all latest values | Form input validation |
| `zip` | Requires corresponding index values from all sources | Combination of values by index | Synchronizing related data |
| `withLatestFrom` | When main source emits value | Main value and latest value from other sources | Combining auxiliary data |
| `forkJoin` | When all sources complete | Last value from each source | Multiple API requests |
| `race` | Only the first source to emit | Only winner stream's values | Timeout, cancellation handling |

### Operator Selection Decision Flow

1. **Want to receive values from all sources at the same time?**
   - Yes → `merge`
   - No → Next

2. **Want to preserve the order of sources?**
   - Yes → `concat`
   - No → Next

3. **Need a combination of the latest values for each source?**
   - Yes → When to combine?
     - For each new value of any source → `combineLatest`
     - For each specific main stream value → `withLatestFrom`
   - No → Next

4. **Need corresponding values in index order?**
   - Yes → `zip`
   - No → Next

5. **Need results after all sources are complete?**
   - Yes → `forkJoin`
   - No → Next

6. **Need only the fastest from multiple alternative sources?**
   - Yes → `race`
   - No → Reexamine the purpose


## Switching Strategy

This is an example of dynamically switching between multiple data sources.

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs';

// Create UI elements
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>Data Source Switching:</h3>';
document.body.appendChild(switchingContainer);

// Create buttons
const source1Button = document.createElement('button');
source1Button.textContent = 'Source 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'Source 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'Source 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// Results display area
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// Three data sources
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map((val) => `Source 1: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '#c8e6c9';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource2() {
  return interval(500).pipe(
    take(8),
    map((val) => `Source 2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('Source 3: A', 'Source 3: B', 'Source 3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// Button click events
const source1Click$ = fromEvent(source1Button, 'click').pipe(map(() => 1));

const source2Click$ = fromEvent(source2Button, 'click').pipe(map(() => 2));

const source3Click$ = fromEvent(source3Button, 'click').pipe(map(() => 3));

// Merge button clicks
merge(source1Click$, source2Click$, source3Click$)
  .pipe(
    // Switch to selected source
    switchMap((sourceId) => {
      // Clear results area
      resultsArea.innerHTML = '';

      // Return selected source
      switch (sourceId) {
        case 1:
          return createSource1();
        case 2:
          return createSource2();
        case 3:
          return createSource3();
        default:
          return of('No source selected');
      }
    })
  )
  .subscribe((value) => {
    // Display result
    const item = document.createElement('div');
    item.textContent = value;
    item.style.padding = '5px';
    item.style.margin = '2px 0';
    item.style.backgroundColor = 'white';
    item.style.borderRadius = '3px';
    resultsArea.appendChild(item);
  });

// Initial message
const initialMessage = document.createElement('div');
initialMessage.textContent =
  'Click a button to select a data source';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);

```

## Conditional Merge

This is an example of combining `merge` and `filter` to select data sources based on conditions.

```ts
import { merge, interval, fromEvent } from 'rxjs';
import {
  map,
  filter,
  takeUntil,
  withLatestFrom,
  startWith,
} from 'rxjs';

// Create UI elements
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>Conditional Merge:</h3>';
document.body.appendChild(conditionalContainer);

// Filter settings
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// Create checkboxes
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = 'Slow Source';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = 'Fast Source';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'Click Events';
filterDiv.appendChild(clickLabel);

// Stop button
const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// Results display area
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// Three data sources
// 1. Slow source (every 1 second)
const slow$ = interval(1000).pipe(map((val) => ({ type: 'slow', value: val })));

// 2. Fast source (every 300 milliseconds)
const fast$ = interval(300).pipe(map((val) => ({ type: 'fast', value: val })));

// 3. Click events
const click$ = fromEvent(document.body, 'click').pipe(
  map((event) => ({
    type: 'click',
    value: {
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY,
    },
  }))
);

// Monitor checkbox state
const slowEnabled$ = fromEvent(slowCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const fastEnabled$ = fromEvent(fastCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const clickEnabled$ = fromEvent(clickCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Stop event
const stop$ = fromEvent(stopButton, 'click');

// Conditional merge
merge(
  // Combine slow source with enabled state
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combine fast source with enabled state
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combine click source with enabled state
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
)
  .pipe(takeUntil(stop$))
  .subscribe((event) => {
    // Display result
    const item = document.createElement('div');

    switch (event.type) {
      case 'slow':
        item.textContent = `Slow Source: ${event.value}`;
        item.style.color = '#1b5e20';
        break;
      case 'fast':
        item.textContent = `Fast Source: ${event.value}`;
        item.style.color = '#0d47a1';
        break;
      case 'click':
        const clickValue = event.value as { x: number; y: number };
        item.textContent = `Click: X=${clickValue.x}, Y=${clickValue.y}`;
        item.style.color = '#bf360c';
        break;
    }

    item.style.padding = '3px';
    item.style.margin = '2px 0';
    conditionalResults.prepend(item); // Display newest at top
  });

```

## Summary of Choosing Combination Operators

| Purpose | Operator | Features |
|------|--------------|------|
| Always sync multiple latest values | `combineLatest` | Always combine latest value of each Observable |
| Get all together after completion | `forkJoin` | Output only last value (once) |
| Process in order synchronously | `zip` | Combine one from each Observable and output |
| Reference other latest values on trigger | `withLatestFrom` | Attach latest value of secondary stream when main stream emits |

## Summary

Combination operators are powerful tools for combining multiple data sources into a single stream. By selecting the appropriate operator, complex asynchronous data flows can be expressed concisely and declaratively.

### Key Points for Mastering Combination Operators

1. **Select the right operator for your use case**: Each operator is optimized for a specific use case. Choose the appropriate operator for your purpose.
2. **Understand when to emit**: The behavior of combination operators is highly dependent on when values are emitted. It is important to understand the emission timing of each operator.
3. **Error Handling Considerations**: Consider the behavior when an error occurs in part of the combined stream (whether the whole thing fails or continues partially).
4. **Knowing completion conditions**: It is also important to understand when the combined stream will complete, and to explicitly complete it using `takeUntil` or similar if necessary.
5. **Take advantage of type safety**: TypeScript allows you to handle combination operators in a type-safe manner. The type benefit is especially great for complex combinations.

Combination operators can be leveraged in many practical scenarios such as UI event handling, multiple API requests, form validation, etc. Mastering these operators will help you unlock the true power of reactive programming in RxJS.

---
Next, let's move on to [Error Handling](/en/guide/error-handling/strategies) to learn how to write more robust RxJS code!
