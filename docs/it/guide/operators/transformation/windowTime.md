---
description: windowTime è un operatore RxJS che può dividere un Observable a intervalli di tempo regolari ed elaborare i valori emessi in ogni intervallo di tempo come un Observable separato.
titleTemplate: ':title | RxJS'
---

# windowTime - Finestra per Tempo

L'operatore `windowTime` raggruppa i valori dell'Observable sorgente **a intervalli regolari** ed emette quel gruppo come un **nuovo Observable**.
Mentre `bufferTime` restituisce un array, `windowTime` restituisce un **Observable&lt;T&gt;**, permettendo di applicare ulteriori operatori a ogni finestra.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Emetti valori ogni 100ms
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Crea finestra ogni 1 secondo
  take(3),          // Solo le prime 3 finestre
  mergeAll()        // Appiattisci ogni finestra
).subscribe(value => {
  console.log('Valore:', value);
});

// Output:
// 1° secondo: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2° secondo: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3° secondo: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- Una nuova finestra (Observable) viene creata ogni tempo specificato (1000ms).
- Ogni finestra può essere elaborata come un Observable indipendente.

[🌐 Documentazione Ufficiale RxJS - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Pattern di Utilizzo Tipici

- **Elaborazione batch basata sul tempo**: I dati vengono elaborati in batch a intervalli regolari
- **Aggregare dati real-time**: Contare il numero di eventi al secondo
- **Monitoraggio prestazioni**: Raccogliere metriche a intervalli regolari
- **Analisi di dati time-series**: Elaborazione statistica per intervallo di tempo

## 🔍 Differenza da bufferTime

| Operatore | Output | Caso d'Uso |
|:---|:---|:---|
| `bufferTime` | **Array (T[])** | Elabora insieme i valori raggruppati |
| `windowTime` | **Observable&lt;T&gt;** | Elaborazione stream diversa per ogni intervallo di tempo |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - Output come array
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Output come Observable
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valore:', value);
  });
});
```

## 🧠 Esempio di Codice Pratico 1: Contare Click al Secondo

Questo è un esempio di conteggio del numero di click su un bottone ogni secondo.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Crea bottone
const button = document.createElement('button');
button.textContent = 'Click';
document.body.appendChild(button);

// Area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento click
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Crea finestra ogni 1 secondo
  map(window$ => {
    ++windowNumber;

    // Conta i click in ogni finestra
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Finestra ${windowNumber}: ${count} click`;
});
```

- Una nuova finestra viene creata ogni secondo.
- Il numero di click in ogni finestra viene contato in tempo reale.

## 🎯 Esempio di Codice Pratico 2: Elaborazione Statistica per Intervallo di Tempo

Questo esempio calcola la somma e la media dei valori per ogni intervallo di tempo.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Elaborazione Statistica per Intervallo di Tempo (ogni 1 secondo)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Finestra</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Conteggio</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Somma</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Media</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Valore casuale
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // Ogni 1 secondo
  take(5),          // Solo 5 finestre
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Converti i valori in ogni finestra in array ed elabora statistiche
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Le statistiche per ogni finestra possono essere calcolate separatamente.
- Elaborazione diversa può essere applicata a ogni finestra.
- Le statistiche sono visualizzate visivamente in formato tabella.

## 📊 Finestre Sovrapposte (windowCreationInterval)

Puoi sovrapporre le finestre specificando `windowCreationInterval` come secondo argomento.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Finestre Sovrapposte</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Lunghezza finestra: 2 secondi
    1000   // Intervallo creazione finestra: 1 secondo
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Finestra ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Valori: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length} valori, ${(result.window - 1)} sec ~ ${(result.window + 1)} sec)`;
  div.appendChild(info);

  output.appendChild(div);

  // Workaround Chrome: Forza rendering
  void output.offsetHeight;
});
```

**Come funziona:**
- **Finestra 1**: Valori da 0 a 2 secondi `[0, 1, 2, ..., 19]` (20 valori)
- **Finestra 2**: Valori da 1 a 3 secondi `[10, 11, 12, ..., 29]` (20 valori) ← Valori 10-19 sovrapposti con Finestra 1
- **Finestra 3**: Valori da 2 a 4 secondi `[20, 21, 22, ..., 39]` (20 valori) ← Valori 20-29 sovrapposti con Finestra 2

- Creare una nuova finestra con un intervallo (1 secondo) più corto della lunghezza della finestra (2 secondi) risulterà in overlap.
- Utile per implementazioni di sliding window.

## 🎯 Esempio Pratico: Monitoraggio Eventi Real-time

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Monitoraggio Movimento Mouse (ogni 5 secondi)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Evento movimento mouse
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // Ogni 5 secondi
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Movimenti mouse: ${result.count} volte`;
  list.insertBefore(item, list.firstChild);

  // Mostra fino a 10 elementi
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Note

### 1. Gestione Subscription delle Finestre

Ogni finestra è un Observable indipendente e deve essere esplicitamente sottoscritta.

```ts
source$.pipe(
  windowTime(1000)
).subscribe(window$ => {
  // I valori non fluiranno a meno che non ti iscrivi alla finestra stessa
  window$.subscribe(value => {
    console.log('Valore:', value);
  });
});
```

Oppure usa `mergeAll()`, `concatAll()`, `switchAll()`, ecc. per appiattire.

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // Unisci tutte le finestre
).subscribe(value => {
  console.log('Valore:', value);
});
```

### 2. Gestione Memoria

Quando si esegue per lunghi periodi, è importante fare unsubscribe correttamente.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Unsubscribe alla distruzione
).subscribe();

// Quando il componente viene distrutto, ecc.
destroy$.next();
destroy$.complete();
```

### 3. Specifica Valore Massimo (maxWindowSize)

Il terzo argomento ti permette di limitare il numero massimo di valori in ogni finestra.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // Lunghezza finestra: 2 secondi
    undefined, // Intervallo creazione finestra: default (senza overlap)
    5          // Conteggio max valori: fino a 5
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Finestra:', values);
  // Contiene solo massimo 5 valori
});
```

## 🆚 Confronto degli Operatori window

| Operatore | Timing del Delimitatore | Caso d'Uso |
|:---|:---|:---|
| `window` | Un altro Observable emette | Partizionamento guidato da eventi |
| `windowTime` | **Intervallo di tempo fisso** | **Partizionamento basato sul tempo** |
| `windowCount` | Conteggio fisso | Partizionamento basato sul conteggio |
| `windowToggle` | Observable di inizio e fine | Controllo dinamico inizio/fine |
| `windowWhen` | Condizione di chiusura dinamica | Condizione di fine diversa per finestra |

## 📚 Operatori Correlati

- [bufferTime](/it/guide/operators/transformation/bufferTime) - Raccogli valori come array (versione array di windowTime)
- [window](/it/guide/operators/transformation/window) - Dividi finestra a timing di Observable diversi
- [windowCount](/it/guide/operators/transformation/windowCount) - Divisione finestre basata sul conteggio
- [windowToggle](/it/guide/operators/transformation/windowToggle) - Controllo finestre con Observable di inizio e fine
- [windowWhen](/it/guide/operators/transformation/windowWhen) - Divisione finestre con condizioni di chiusura dinamiche

## Riepilogo

L'operatore `windowTime` è uno strumento potente per dividere stream su base temporale e trattare ogni intervallo di tempo come un Observable indipendente.

- ✅ Crea automaticamente finestre a intervalli regolari
- ✅ Elaborazione diversa può essere applicata a ogni finestra
- ✅ Supporta sliding window (overlap)
- ✅ Ideale per aggregazione e analisi dati real-time
- ⚠️ Richiede gestione subscription
- ⚠️ Fai attenzione alla gestione memoria
