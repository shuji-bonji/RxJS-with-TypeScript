---
description: zipWith is an RxJS combination operator that pairs the original Observable with other Observables in corresponding order. It is the Pipeable Operator version of Creation Function zip, and is ideal for situations where order is important, such as quiz games (questions and answers), task assignments (users and tasks), seat assignments (passengers and seat numbers), etc. It is useful when you want to transform and process the main stream while pairing it with other streams.
titleTemplate: ':title | RxJS'
---

# zipWith - Pairing Based on Order (Within Pipeline)

The `zipWith` operator pairs the original Observable with the specified other Observables **in corresponding order** to form a new stream.
This is the Pipeable Operator version of the Creation Function `zip`.

## 🔰 Basic Syntax and Usage

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C', 'D');
const source2$ = interval(500).pipe(take(4)); // 0, 1, 2, 3

source1$
  .pipe(
    zipWith(source2$),
    map(([letter, num]) => `${letter}${num}`)
  )
  .subscribe(console.log);

// Output:
// A0 (after 500ms)
// B1 (after 1000ms)
// C2 (after 1500ms)
// D3 (after 2000ms)
```

- Pairs **values in corresponding order** from each Observable, one at a time.
- **Waits until all Observables have emitted values in corresponding order** before outputting the pair.
- When any Observable completes, the entire stream completes.

[🌐 RxJS Official Documentation - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## 💡 Typical Usage Patterns

- **Quiz game question and answer pairing**: Pairing sequential questions with user answers
- **Task assignment**: Pairing user lists with task lists in sequence
- **Seat assignment**: Pairing passengers with seat numbers in sequence
- **Consolidate parallel processing results**: Combine results of multiple API calls in order


## 🧠 Practical Code Example (with UI)

Example of a quiz game where questions and user answers are paired in order and scored.

```ts
import { fromEvent, of, from } from 'rxjs';
import { zipWith, map, take, scan } from 'rxjs';

// Build the UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>zipWith Practical Example: Quiz Game</h3>
  <div id="question" style="font-size: 18px; margin: 10px 0;">Loading questions...</div>
  <div>
    <button id="answer-a">A</button>
    <button id="answer-b">B</button>
    <button id="answer-c">C</button>
  </div>
  <div id="result" style="margin-top: 10px;"></div>
  <div id="score" style="margin-top: 10px; font-weight: bold;"></div>
`;
document.body.appendChild(container);

const questionDiv = document.getElementById('question')!;
const resultDiv = document.getElementById('result')!;
const scoreDiv = document.getElementById('score')!;

const buttonA = document.getElementById('answer-a') as HTMLButtonElement;
const buttonB = document.getElementById('answer-b') as HTMLButtonElement;
const buttonC = document.getElementById('answer-c') as HTMLButtonElement;

// Question list (with correct answers)
interface Question {
  id: number;
  text: string;
  correct: string;
}

const questions: Question[] = [
  { id: 1, text: 'What does "Rx" in RxJS stand for?', correct: 'A' },
  { id: 2, text: 'What does Observable represent?', correct: 'B' },
  { id: 3, text: 'What does subscribe do?', correct: 'C' }
];

// Question stream
const questions$ = of(...questions);

let currentQuestionIndex = 0;
questions$.subscribe(q => {
  if (currentQuestionIndex === 0) {
    questionDiv.textContent = `Q${q.id}: ${q.text}`;
  }
});

// User answer stream (button clicks)
const getAnswer = () => new Promise<string>((resolve) => {
  const handleClick = (answer: string) => {
    resolve(answer);
    buttonA.removeEventListener('click', handleA);
    buttonB.removeEventListener('click', handleB);
    buttonC.removeEventListener('click', handleC);
  };
  const handleA = () => handleClick('A');
  const handleB = () => handleClick('B');
  const handleC = () => handleClick('C');
  buttonA.addEventListener('click', handleA);
  buttonB.addEventListener('click', handleB);
  buttonC.addEventListener('click', handleC);
});

const answers$ = from(
  Promise.all(questions.map(() => getAnswer()))
);

// Pair and grade questions with answers
questions$
  .pipe(
    zipWith(answers$),
    map(([question, answer]) => ({
      question: question.text,
      answer,
      correct: question.correct,
      isCorrect: answer === question.correct
    })),
    scan((acc, result) => ({
      ...result,
      totalScore: acc.totalScore + (result.isCorrect ? 1 : 0)
    }), { totalScore: 0 } as any)
  )
  .subscribe((result) => {
    const status = result.isCorrect ? '✅ Correct' : '❌ Incorrect';
    resultDiv.innerHTML += `<div>${status}: ${result.question} - Your answer: ${result.answer}</div>`;
    scoreDiv.textContent = `Current score: ${result.totalScore} / ${currentQuestionIndex + 1}`;
    currentQuestionIndex++;

    // Show next question
    if (currentQuestionIndex < questions.length) {
      questionDiv.textContent = `Q${questions[currentQuestionIndex].id}: ${questions[currentQuestionIndex].text}`;
    } else {
      questionDiv.textContent = 'All questions completed!';
      buttonA.disabled = true;
      buttonB.disabled = true;
      buttonC.disabled = true;
    }
  });
```

- Each time a user answers, it is **paired with the corresponding question** and scored.
- Order is guaranteed, so the correspondence is maintained: **Answer 1 for Question 1, Answer 2 for Question 2**, and so on.


## 🔄 Difference from Creation Function `zip`

### Basic Differences

| | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **Usage Location** | Used as independent function | Used within `.pipe()` chain |
| **Syntax** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **First Stream** | Treats all equally | Treats as main stream |
| **Advantage** | Simple and readable | Easy to combine with other operators |

### Specific Usage Examples

**Creation Function is Recommended for Simple Pairing Only**

```ts
import { zip, of } from 'rxjs';

const names$ = of('Alice', 'Bob', 'Charlie');
const ages$ = of(25, 30, 35);
const cities$ = of('Tokyo', 'Osaka', 'Kyoto');

// Simple and readable
zip(names$, ages$, cities$).subscribe(([name, age, city]) => {
  console.log(`${name} (${age} years old) - ${city}`);
});
// Output:
// Alice (25 years old) - Tokyo
// Bob (30 years old) - Osaka
// Charlie (35 years old) - Kyoto
```

**Pipeable Operator is Recommended When Adding Transformation Processing to Main Stream**

```ts
import { of } from 'rxjs';
import { zipWith, map, filter } from 'rxjs';

const users$ = of(
  { id: 1, name: 'Alice', active: true },
  { id: 2, name: 'Bob', active: false },
  { id: 3, name: 'Charlie', active: true }
);

const tasks$ = of('Task A', 'Task B', 'Task C');

// ✅ Pipeable Operator version - completed in one pipeline
users$
  .pipe(
    filter(user => user.active),    // Active users only
    map(user => user.name),         // Extract name only
    zipWith(tasks$)                 // Pair with tasks
  )
  .subscribe(([user, task]) => {
    console.log(`Assign ${task} to ${user}`);
  });
// Output:
// Assign Task A to Alice
// Assign Task B to Charlie

// ❌ Creation Function version - becomes verbose
import { zip } from 'rxjs';
zip(
  users$.pipe(
    filter(user => user.active),
    map(user => user.name)
  ),
  tasks$
).subscribe(([user, task]) => {
  console.log(`Assign ${task} to ${user}`);
});
```

### Summary

- **`zip`**: Optimal for simply pairing multiple streams
- **`zipWith`**: Optimal when you want to transform/process the main stream while pairing it with other streams


## ⚠️ Important Notes

### Completion Timing

When any Observable completes, the entire stream completes.

```ts
import { of, interval } from 'rxjs';
import { zipWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    interval(1000).pipe(take(2)),  // Emits only 2 values
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Complete')
});
// Output: [1, 0] → [2, 1] → ✅ Complete
// * interval$ emitted only 2 values and completed, so 3 is not paired
```

### Synchronization of Emission Timing

`zipWith` waits **until all Observables have emitted values in corresponding order**.

```ts
import { interval } from 'rxjs';
import { zipWith, take, map } from 'rxjs';

const fast$ = interval(100).pipe(take(5), map(i => `Fast: ${i}`));
const slow$ = interval(1000).pipe(take(5), map(i => `Slow: ${i}`));

fast$
  .pipe(zipWith(slow$))
  .subscribe(console.log);
// Output (every 1 second):
// ['Fast: 0', 'Slow: 0']
// ['Fast: 1', 'Slow: 1']
// ['Fast: 2', 'Slow: 2']
// ['Fast: 3', 'Slow: 3']
// ['Fast: 4', 'Slow: 4']
// * fast$ is fast, but waits for slow$ to emit, so pairs are output every second
```

### Difference from combineLatestWith

`combineLatestWith` **always combines** the latest values, whereas `zipWith` **pairs based on order**.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(100).pipe(take(3)); // 0, 1, 2
const source2$ = interval(200).pipe(take(2)); // 0, 1

// zipWith: Pair by order
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Output: [0, 0] → [1, 1] → Complete
// * Since source2$ completed, source1$'s 2 is not paired

// combineLatestWith: Combine latest values
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Output: [0, 0] → [1, 0] → [1, 1] → [2, 1]
// * Outputs latest value combination each time either emits
```

### Error Handling

If an error occurs in any Observable, the entire stream terminates with an error.

```ts
import { throwError, of } from 'rxjs';
import { zipWith, catchError } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    throwError(() => new Error('Error occurred')).pipe(
      catchError(err => of('Error recovered'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: [1, 'Error recovered']
```


## 📚 Related Operators

- **[zip](/pt/guide/creation-functions/combination/zip)** - Creation Function version
- **[combineLatestWith](/pt/guide/operators/combination/combineLatestWith)** - Always combine latest values
- **[withLatestFrom](/pt/guide/operators/combination/withLatestFrom)** - Combine only when main stream emits
