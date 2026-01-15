---
description: zipWith è un operatore di combinazione RxJS che accoppia l'Observable originale con altri Observable in ordine corrispondente. È la versione Pipeable Operator della Funzione di Creazione zip, ed è ideale per situazioni in cui l'ordine è importante, come giochi quiz (domande e risposte), assegnazioni di task (utenti e task), assegnazioni di posti (passeggeri e numeri di posto), ecc. È utile quando vuoi trasformare ed elaborare lo stream principale mentre lo accoppi con altri stream.
titleTemplate: ':title | RxJS'
---

# zipWith - Accoppia per Ordine

L'operatore `zipWith` accoppia l'Observable originale con gli altri Observable specificati **in ordine corrispondente** per formare un nuovo stream.
Questa è la versione Pipeable Operator della Funzione di Creazione `zip`.

## 🔰 Sintassi e Utilizzo Base

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
// A0 (dopo 500ms)
// B1 (dopo 1000ms)
// C2 (dopo 1500ms)
// D3 (dopo 2000ms)
```

- Accoppia **valori in ordine corrispondente** da ogni Observable, uno alla volta.
- **Attende fino a quando tutti gli Observable hanno emesso valori in ordine corrispondente** prima di emettere la coppia.
- Quando qualsiasi Observable completa, l'intero stream completa.

[🌐 Documentazione Ufficiale RxJS - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## 💡 Pattern di Utilizzo Tipici

- **Accoppiamento domande e risposte in giochi quiz**: Accoppiamento domande sequenziali con risposte utente
- **Assegnazione task**: Accoppiamento liste utenti con liste task in sequenza
- **Assegnazione posti**: Accoppiamento passeggeri con numeri di posto in sequenza
- **Consolidare risultati elaborazione parallela**: Combinare risultati di più chiamate API in ordine


## 🧠 Esempio di Codice Pratico (con UI)

Esempio di un gioco quiz dove domande e risposte utente vengono accoppiate in ordine e valutate.

```ts
import { fromEvent, of, from } from 'rxjs';
import { zipWith, map, take, scan } from 'rxjs';

// Costruisci la UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>Esempio Pratico zipWith: Gioco Quiz</h3>
  <div id="question" style="font-size: 18px; margin: 10px 0;">Caricamento domande...</div>
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

// Lista domande (con risposte corrette)
interface Question {
  id: number;
  text: string;
  correct: string;
}

const questions: Question[] = [
  { id: 1, text: 'Cosa significa "Rx" in RxJS?', correct: 'A' },
  { id: 2, text: 'Cosa rappresenta un Observable?', correct: 'B' },
  { id: 3, text: 'Cosa fa subscribe?', correct: 'C' }
];

// Stream domande
const questions$ = of(...questions);

let currentQuestionIndex = 0;
questions$.subscribe(q => {
  if (currentQuestionIndex === 0) {
    questionDiv.textContent = `D${q.id}: ${q.text}`;
  }
});

// Stream risposte utente (click pulsanti)
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

// Accoppia e valuta domande con risposte
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
    const status = result.isCorrect ? '✅ Corretto' : '❌ Sbagliato';
    resultDiv.innerHTML += `<div>${status}: ${result.question} - La tua risposta: ${result.answer}</div>`;
    scoreDiv.textContent = `Punteggio attuale: ${result.totalScore} / ${currentQuestionIndex + 1}`;
    currentQuestionIndex++;

    // Mostra prossima domanda
    if (currentQuestionIndex < questions.length) {
      questionDiv.textContent = `D${questions[currentQuestionIndex].id}: ${questions[currentQuestionIndex].text}`;
    } else {
      questionDiv.textContent = 'Tutte le domande completate!';
      buttonA.disabled = true;
      buttonB.disabled = true;
      buttonC.disabled = true;
    }
  });
```

- Ogni volta che un utente risponde, viene **accoppiata con la domanda corrispondente** e valutata.
- L'ordine è garantito, quindi la corrispondenza viene mantenuta: **Risposta 1 per Domanda 1, Risposta 2 per Domanda 2**, e così via.


## 🔄 Differenza dalla Funzione di Creazione `zip`

### Differenze Base

| | `zip` (Funzione di Creazione) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **Posizione di Utilizzo** | Usato come funzione indipendente | Usato all'interno della catena `.pipe()` |
| **Sintassi** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Primo Stream** | Tratta tutti ugualmente | Tratta come stream principale |
| **Vantaggio** | Semplice e leggibile | Facile da combinare con altri operatori |

### Esempi di Utilizzo Specifici

**La Funzione di Creazione è Raccomandata per Solo Accoppiamento Semplice**

```ts
import { zip, of } from 'rxjs';

const names$ = of('Alice', 'Bob', 'Charlie');
const ages$ = of(25, 30, 35);
const cities$ = of('Tokyo', 'Osaka', 'Kyoto');

// Semplice e leggibile
zip(names$, ages$, cities$).subscribe(([name, age, city]) => {
  console.log(`${name} (${age} anni) - ${city}`);
});
// Output:
// Alice (25 anni) - Tokyo
// Bob (30 anni) - Osaka
// Charlie (35 anni) - Kyoto
```

**Il Pipeable Operator è Raccomandato Quando si Aggiunge Elaborazione di Trasformazione allo Stream Principale**

```ts
import { of } from 'rxjs';
import { zipWith, map, filter } from 'rxjs';

const users$ = of(
  { id: 1, name: 'Alice', active: true },
  { id: 2, name: 'Bob', active: false },
  { id: 3, name: 'Charlie', active: true }
);

const tasks$ = of('Task A', 'Task B', 'Task C');

// ✅ Versione Pipeable Operator - completato in una pipeline
users$
  .pipe(
    filter(user => user.active),    // Solo utenti attivi
    map(user => user.name),         // Estrai solo nome
    zipWith(tasks$)                 // Accoppia con task
  )
  .subscribe(([user, task]) => {
    console.log(`Assegna ${task} a ${user}`);
  });
// Output:
// Assegna Task A a Alice
// Assegna Task B a Charlie

// ❌ Versione Funzione di Creazione - diventa prolissa
import { zip } from 'rxjs';
zip(
  users$.pipe(
    filter(user => user.active),
    map(user => user.name)
  ),
  tasks$
).subscribe(([user, task]) => {
  console.log(`Assegna ${task} a ${user}`);
});
```

### Riepilogo

- **`zip`**: Ottimale per accoppiare semplicemente più stream
- **`zipWith`**: Ottimale quando vuoi trasformare/elaborare lo stream principale mentre lo accoppi con altri stream


## ⚠️ Note Importanti

### Timing di Completamento

Quando qualsiasi Observable completa, l'intero stream completa.

```ts
import { of, interval } from 'rxjs';
import { zipWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    interval(1000).pipe(take(2)),  // Emette solo 2 valori
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Completo')
});
// Output: [1, 0] → [2, 1] → ✅ Completo
// * interval$ ha emesso solo 2 valori e completato, quindi 3 non viene accoppiato
```

### Sincronizzazione del Timing di Emissione

`zipWith` attende **fino a quando tutti gli Observable hanno emesso valori in ordine corrispondente**.

```ts
import { interval } from 'rxjs';
import { zipWith, take, map } from 'rxjs';

const fast$ = interval(100).pipe(take(5), map(i => `Veloce: ${i}`));
const slow$ = interval(1000).pipe(take(5), map(i => `Lento: ${i}`));

fast$
  .pipe(zipWith(slow$))
  .subscribe(console.log);
// Output (ogni 1 secondo):
// ['Veloce: 0', 'Lento: 0']
// ['Veloce: 1', 'Lento: 1']
// ['Veloce: 2', 'Lento: 2']
// ['Veloce: 3', 'Lento: 3']
// ['Veloce: 4', 'Lento: 4']
// * fast$ è veloce, ma aspetta che slow$ emetta, quindi le coppie vengono emesse ogni secondo
```

### Differenza da combineLatestWith

`combineLatestWith` **combina sempre** gli ultimi valori, mentre `zipWith` **accoppia in base all'ordine**.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(100).pipe(take(3)); // 0, 1, 2
const source2$ = interval(200).pipe(take(2)); // 0, 1

// zipWith: Accoppia per ordine
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Output: [0, 0] → [1, 1] → Completo
// * Poiché source2$ ha completato, il 2 di source1$ non viene accoppiato

// combineLatestWith: Combina ultimi valori
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Output: [0, 0] → [1, 0] → [1, 1] → [2, 1]
// * Emette combinazione ultimi valori ogni volta che uno dei due emette
```

### Gestione Errori

Se si verifica un errore in qualsiasi Observable, l'intero stream termina con un errore.

```ts
import { throwError, of } from 'rxjs';
import { zipWith, catchError } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    throwError(() => new Error('Si è verificato un errore')).pipe(
      catchError(err => of('Errore recuperato'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Output: [1, 'Errore recuperato']
```


## 📚 Operatori Correlati

- **[zip](/it/guide/creation-functions/combination/zip)** - Versione Funzione di Creazione
- **[combineLatestWith](/it/guide/operators/combination/combineLatestWith)** - Combina sempre ultimi valori
- **[withLatestFrom](/it/guide/operators/combination/withLatestFrom)** - Combina solo quando lo stream principale emette
