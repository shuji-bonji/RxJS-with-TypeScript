---
description: "L'operatore findIndex è un operatore di filtraggio di RxJS che restituisce l'indice del primo valore che soddisfa la condizione. Se non viene trovato, restituisce -1."
---

# findIndex - ottiene l'indice che corrisponde alla condizione

L'operatore findIndex restituisce **l'indice del primo valore che corrisponde alla condizione** e completa immediatamente il flusso. Restituisce `-1` se non viene trovato alcun valore.

## 🔰 Sintassi e utilizzo di base

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Uscita.: 4(primo pari8indice del primo pari)
```

**Flusso delle operazioni**:.
1. 1 (indice 0) → dispari, salto
2. 3 (indice 1) → dispari, saltare
3. 5 (indice 2) → dispari, saltare
4. 7 (indice 3) → Dispari, saltare
5. 8 (indice 4) → numero pari, uscita indice 4 e completamento

[🌐 Documentazione ufficiale RxJS - findIndex](https://rxjs.dev/api/operators/findIndex)

## 💡 Tipico modello di utilizzo.

- **Posizionamento in un array**: ottenere la posizione di un elemento che soddisfa una condizione specifica.
- **Controllo dell'ordine**: quante volte appare un elemento che soddisfa una determinata condizione.
- **Riordino dei dati**: elaborazione utilizzando le informazioni dell'indice.
- **Controllo dell'esistenza**: verifica l'esistenza di un elemento controllando se è -1 o no.

## 🧠 Esempio pratico di codice 1: Ricerca in un elenco di attività

Questo è un esempio di ricerca della posizione di un'attività con condizioni specifiche da un elenco di attività.

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'Risposta all'e-mail', priority: 'low', completed: true },
  { id: 2, title: 'Preparazione del documento', priority: 'medium', completed: true },
  { id: 3, title: 'Preparazione della riunione', priority: 'high', completed: false },
  { id: 4, title: 'Revisione del codice', priority: 'high', completed: false },
  { id: 5, title: 'Aggiornamento del documento', priority: 'low', completed: false }
];

// UICreare
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Ricerca attività';
container.appendChild(title);

// Visualizzazione dell'elenco dei compiti
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// Pulsante di ricerca
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Ricerca del primo compito non completato';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Ricerca del primo compito ad alta priorità';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Ricerca del primo compito non completato
// NB.: In origine, lo schema raccomandato era di appiattire con switchMap Lo schema consigliato è di appiattire con
// In questo caso, la priorità di leggibilità è data a subscribe annidato (nel codice di produzione switchMap raccomandato).
fromEvent(button1, 'click').subscribe(() => {
  // Nidificazione subscribe: In origine, lo schema raccomandato era di appiattire con switchMap L'appiattimento con è consigliato
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Trovato in</strong><br>
        Posizione: Indice ${index}<br>
        Compito: ${task.title}<br>
        Priorità: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Compito incompiuto non trovato';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Ricerca del primo compito ad alta priorità
// NB.: In origine, lo schema raccomandato era di appiattire con switchMap Lo schema raccomandato (nel codice di produzione) è quello di appiattire con switchMap raccomandato).
fromEvent(button2, 'click').subscribe(() => {
  // Nidificazione subscribe: In origine, lo schema raccomandato era di appiattire con switchMap L'appiattimento con è consigliato
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Trovato in</strong><br>
        Posizione: Indice ${index}<br>
        Compito: ${task.title}<br>
        Stato di completamento: ${task.completed ? 'Completato' : 'Non completato'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Non sono stati trovati compiti ad alta priorità';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Trova la posizione del primo task nell'elenco dei task che soddisfa la condizione.
- Se non viene trovata, viene restituito `-1`.

## 🎯 Esempio pratico di codice 2: rilevamento della posizione dei dati in tempo reale

Questo esempio rileva la posizione del primo valore del flusso che soddisfa la condizione.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UICreare
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Ricerca dei dati in tempo reale';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Ricerca di posizioni con un valore maggiore o uguale a...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// Generazione di valori casuali (0~100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Visualizzazione dei dati
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Valore: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50Cerca nell'indice il primo valore di più di
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50Valore maggiore o uguale a trovato<br>
      Posizione: Indice ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50Non sono stati trovati valori maggiori o uguali a';
    result.style.color = 'orange';
  }
});
```

- Rileva la posizione del primo valore superiore a 50 da valori casuali generati ogni 0,5 secondi.
- L'evidenziazione è utilizzata per chiarezza visiva.

## 🆚 Confronto con operatori simili

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Restituisce l'indice del primo valore che soddisfa la condizione
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Uscita.: 2Restituisce l'indice del primo valore che soddisfa la condizione30indice del primo pari)

// find: Restituisce il primo valore che soddisfa la condizione
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Uscita.: 30

// elementAt: Restituisce il valore all'indice specificato
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Uscita.: 30
```

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Uscita.: 4(primo pari8indice del primo pari)

## 🔄 Confronto con Array.findIndex( di JavaScript)

RxJS `findIndex` si comporta in modo simile al metodo array di JavaScript `Array.prototype.findIndex()`.

```ts
// JavaScript Array di
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS (restituisce il primo valore all'indice specificato che soddisfa la condizione) Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Differenze principali**.
- **Array**: restituisce il risultato in modo sincrono e immediato.
- Observable**: asincrono, attende che i valori vengano inviati dallo stream.

## ⚠️ Note.

### 1. restituisce -1 se non viene trovato

Se nessun valore soddisfa la condizione, restituisce `-1` invece di un errore.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Non è stato trovato alcun valore che soddisfa la condizione');
  } else {
    console.log(`Indice: ${index}`);
  }
});
// Uscita.: Non è stato trovato alcun valore che soddisfa la condizione
```

### 2. completare quando viene trovato per la prima volta.

Il flusso viene completato non appena viene trovato il primo valore che soddisfa la condizione.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Valore: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Indice: ${index}`);
});
// Uscita.:
// Valore: 0
// Valore: 1
// Valore: 2
// Valore: 3
// Indice: 3
```

### 3. Sicurezza dei tipi in TypeScript

findIndex restituisce sempre il tipo `number`.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index è un array di number tipo
  if (index !== -1) {
    console.log(`Il primo utente inattivo è l'indice ${index} è.`);
  }
});
// Uscita.: Il primo utente inattivo è l'indice 1 è.
```

### 4. l'indice parte da 0

Come per gli array, gli indici iniziano da 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Uscita.: 0(primo elemento)
```

## 📚 Operatori correlati.

- **[find](. /find)** - Ottiene il primo valore che soddisfa la condizione.
- **[elementAt](. /elementAt)** - Ottiene il valore all'indice specificato.
- **[first](. /first)** - Ottiene il primo valore.
- **[filter](. /filtro)** - ottiene tutti i valori che soddisfano la condizione.

## Riepilogo.

L'operatore findIndex restituisce l'indice del primo valore che soddisfa la condizione.

- Comportamento simile a `Array.findIndex()` di JavaScript.
- Ideale quando sono necessarie informazioni sull'indice.
- ✅ Restituisce `-1' se non viene trovato (non è un errore)
- ✅ Completa immediatamente quando viene trovato
- ⚠️ Il valore restituito è sempre di tipo `number` (-1 o un intero maggiore o uguale a 0)
- ⚠️ Utilizza `find` se è necessario il valore stesso
