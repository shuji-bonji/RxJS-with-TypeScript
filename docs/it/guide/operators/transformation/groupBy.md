---
description: "L'operatore groupBy raggruppa i valori del flusso in base a una chiave specificata e crea un Observable separato per ogni gruppo. Implementazione sicura e casi d'uso pratici in TypeScript, come l'aggregazione per categoria, l'elaborazione per utente e la categorizzazione dei dati."
---

# groupBy - raggruppa i valori in base alla chiave

L'operatore groupBy raggruppa i valori emessi da un flusso in base a una chiave specificata e restituisce ogni gruppo come un Observable separato.
È utile se si desidera categorizzare i dati o applicare un'elaborazione diversa a ciascun gruppo.

## 🔰 Sintassi e uso di base

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: 'Taro', age: 25 },
  { name: 'Hanako', age: 30 },
  { name: 'Jiro', age: 25 },
  { name: 'Misaki', age: 30 },
  { name: 'Kenta', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // Raggruppati per età
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Età ${result.age}:`, result.people);
});

// Uscita:
// Età 25: [{name: 'Taro', age: 25}, {name: 'Jiro', age: 25}]
// Età 30: [{name: 'Hanako', age: 30}, {name: 'Misaki', age: 30}]
// Età 35: [{name: 'Kenta', age: 35}]
```

- groupBy(person => person.age)` per raggruppare l'età come chiave.
- Ogni gruppo è trattato come un `Observable raggruppato` e la chiave del gruppo è accessibile tramite la proprietà `key`.
- Elaborare ogni Observable raggruppato con `mergeMap`.

[🌐 Documentazione ufficiale di RxJS - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Tipici modelli di utilizzo

- Categorizzazione dei dati per categoria
- Elaborazione aggregata per gruppo
- Elaborazione di log ed eventi per tipo
- Raggruppamento e trasformazione dei dati

## 🧠 Esempi pratici di codice (con UI)

Questo esempio mostra il numero di pezzi raggruppati per colore quando si fa clic su un pulsante.

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// Pulsante di creazione
const colors = ['Rosso', 'Blu', 'Verde', 'Giallo'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = 'Aggregato.';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// Creare un'area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Registra i colori cliccati
const clicks: string[] = [];

// Eventi di clic dei pulsanti a colori
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `Colore selezionato: ${clicks.join(', ')}`;
  }
});

// Raggruppati al clic del pulsante di aggregazione
fromEvent(calculateButton, 'click').pipe(
  switchMap(() =>
    from(clicks).pipe(
      groupBy(color => color),
      mergeMap(group =>
        group.pipe(
          toArray(),
          map(items => ({ color: group.key, count: items.length }))
        )
      ),
      toArray()
    )
  )
).subscribe(results => {
  if (results.length === 0) {
    output.innerHTML = '<p>Nessun colore ancora selezionato</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count}Tempi`)
    .join('<br>');
  output.innerHTML = `<h3>Risultato dell'aggregazione</h3>${resultText}`;
});
```

- Fare clic sul pulsante Colore per selezionare un colore
- Raggruppare per colore con il pulsante `totalizza` e visualizzare il numero di pezzi.
- Raggruppare per colore con il pulsante `groupBy` e contare il numero di elementi in ogni gruppo.

## 🎯 Esempio di conteggio per categoria

Questo è un esempio di raggruppamento dei prodotti per categoria e di calcolo del totale per ogni categoria.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'Mele', category: 'Frutta', price: 150 },
  { name: 'Arance mandarino', category: 'Frutta', price: 100 },
  { name: 'Carote', category: 'Verdura', price: 80 },
  { name: 'Pomodori', category: 'Verdura', price: 120 },
  { name: 'Latte', category: 'Prodotti caseari', price: 200 },
  { name: 'Formaggio', category: 'Prodotti caseari', price: 300 },
];

from(products).pipe(
  groupBy(product => product.category),
  mergeMap(group =>
    group.pipe(
      reduce((total, product) => total + product.price, 0),
      map(total => ({ category: group.key, total }))
    )
  )
).subscribe(result => {
  console.log(`${result.category}: ${result.total}Cerchio`);
});

// Uscita:
// Frutta: 250Cerchio
// Verdura: 200Cerchio
// Prodotti caseari: 500Cerchio
```

## 🎯 Esempio di utilizzo del selettore di elementi.

Quando si raggruppa, i valori possono anche essere convertiti.

```ts
import { from } from 'rxjs';
import { groupBy, map, mergeMap, toArray } from 'rxjs';

interface Student {
  name: string;
  grade: number;
  score: number;
}

const students: Student[] = [
  { name: 'Taro', grade: 1, score: 85 },
  { name: 'Hanako', grade: 2, score: 92 },
  { name: 'Jiro', grade: 1, score: 78 },
  { name: 'Misaki', grade: 2, score: 88 },
];

from(students).pipe(
  groupBy(
    student => student.grade,           // Selettore di chiavi
    student => student.name             // Selettore di elementi (contiene solo nomi)
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`${result.grade}Anno studente:`, result.students.join(', '));
});

// Uscita:
// 1Anno studente: Taro, Jiro
// 2Anno studente: Hanako, Misaki
```

- 1° argomento: selettore di chiavi (criteri per il raggruppamento)
- Secondo argomento: selettore di elementi (valori da memorizzare nel gruppo).

## 🎯 Uso di groupBy sicuro per il tipo

Questo è un esempio di utilizzo dell'inferenza di tipo di TypeScript.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

type LogLevel = 'info' | 'warning' | 'error';

interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: number;
}

const logs: LogEntry[] = [
  { level: 'info', message: 'Avvio dell'app', timestamp: 1000 },
  { level: 'warning', message: 'Messaggio di avviso', timestamp: 2000 },
  { level: 'error', message: 'Si è verificato un errore', timestamp: 3000 },
  { level: 'info', message: 'Elaborazione completata', timestamp: 4000 },
  { level: 'error', message: 'Errore di connessione', timestamp: 5000 },
];

from(logs).pipe(
  groupBy(log => log.level),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(entries => ({
        level: group.key,
        count: entries.length,
        messages: entries.map(e => e.message)
      }))
    )
  )
).subscribe(result => {
  console.log(`[${result.level.toUpperCase()}] ${result.count}Caso`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Uscita:
// [INFO] 2Caso
//   - Avvio dell'app
//   - Elaborazione completata
// [WARNING] 1Caso
//   - Messaggio di avviso
// [ERROR] 2Caso
//   - Si è verificato un errore
//   - Errore di connessione
```

## 🎯 Applicare processi diversi a gruppi diversi

Questo è un esempio di applicazione di trattamenti diversi a ciascun gruppo.

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: 'Attività urgente' },
  { id: 2, priority: 'low', name: 'Attività rimandata' },
  { id: 3, priority: 'high', name: 'Attività importanti' },
  { id: 4, priority: 'medium', name: 'Attività normali' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // I tempi di ritardo sono impostati in base alla priorità
    const delayTime =
      group.key === 'high' ? 0 :
      group.key === 'medium' ? 1000 :
      2000;

    return group.pipe(
      delay(delayTime),
      map(task => ({ ...task, processedAt: Date.now() }))
    );
  })
).subscribe(task => {
  console.log(`[${task.priority}] ${task.name} Elaborazione`);
});

// Uscita (in ordine di priorità):
// [high] Attività urgente Elaborazione
// [high] Attività importanti Elaborazione
// (1(dopo 1,5 secondi)
// [medium] Attività normali Elaborazione
// (ulteriore)1(dopo 1,5 secondi)
// [low] Attività rimandata Elaborazione
```

## ⚠️ Note.

### Gestione delle sottoscrizioni per il gruppo Observable.

groupBy crea un Observable per ogni gruppo. Questi Observable possono causare perdite di memoria se non sono correttamente sottoscritti (subscribe).

```ts
// ❌ Esempio negativo: Il gruppoObservablenon si iscrive a
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // Il gruppoObservableNon sottoscritto
  console.log('Il gruppo:', group.key);
});
```

**Misure**: elaborare sempre ogni gruppo con `mergeMap`, `concatMap`, `switchMap`, ecc.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Buon esempio: Ogni gruppo viene gestito in modo appropriato
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Generazione dinamica di gruppi

Ogni volta che compare una nuova chiave viene creato un nuovo Observable di gruppo. Occorre prestare attenzione se ci sono molti tipi di chiave.

```ts
// Esempio di un numero potenzialmente infinito di tipi di chiavi
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // Chiavi diverse ogni volta
).subscribe(); // Pericolo di perdite di memoria
```

## 📚 Operatori correlati.

- [`partition`](https://rxjs.dev/api/index/function/partition) - divide in due Observable per condizione.
- [`reduce`](. /reduce) - Ottiene il risultato finale aggregato.
- [`scan`](. /scan) - Aggregazione cumulativa.
- [`toArray`](. /utility/toArray) - Combina tutti i valori in un array.

## Riepilogo.

L'operatore groupBy può raggruppare i valori in un flusso in base a chiavi e **trattare ogni gruppo come un Observable separato**. Ciò è molto utile per l'elaborazione di dati complessi, come la categorizzazione dei dati, l'aggregazione per categoria e l'elaborazione di ciascun gruppo in modo diverso. Tuttavia, ogni Observable di gruppo deve essere sottoscritto in modo appropriato e di solito viene usato insieme a una mergeMap o simili.
