---
description: L'operatore pairwise è un operatore RxJS che emette due valori consecutivi come un array di coppie, ed è utilizzato per confrontare il valore precedente con quello corrente o per calcolare la differenza.
titleTemplate: ':title | RxJS'
---

# pairwise - Elabora Due Valori Consecutivi come Coppia

L'operatore `pairwise` **accoppia due valori consecutivi emessi da uno stream come un array `[valore precedente, valore corrente]` e li emette insieme**.
Questo è utile per confrontare il valore precedente con quello corrente o per calcolare l'ammontare del cambiamento.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Output:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- Il primo valore (0) non viene emesso da solo, ma viene emesso come `[0, 1]` quando arriva il secondo valore (1).
- Viene sempre emessa una coppia di **valore precedente e valore corrente**.

[🌐 Documentazione Ufficiale RxJS - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Pattern di Utilizzo Tipici

- Calcolo dell'ammontare del movimento del mouse o del touch
- Calcolo dell'ammontare del cambiamento (differenza) in prezzi o valori
- Rilevamento cambio di stato (confronto stato precedente e stato corrente)
- Determinazione della direzione dello scroll

## 🧠 Esempio di Codice Pratico (con UI)

Questo esempio visualizza la direzione e l'ammontare del movimento del mouse.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Evento movimento mouse
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'Destra' : deltaX < 0 ? 'Sinistra' : 'Fermo';

  output.innerHTML = `
    Precedente: (${prev.x}, ${prev.y})<br>
    Corrente: (${curr.x}, ${curr.y})<br>
    Movimento: Δx=${deltaX}, Δy=${deltaY}<br>
    Direzione: ${direction}
  `;
});
```

- Quando il mouse viene mosso, vengono visualizzate le coordinate precedenti e correnti e l'ammontare del movimento.
- Con `pairwise`, le coordinate precedenti e correnti possono essere ottenute automaticamente in coppia.

## 🎯 Esempio di Calcolo dell'Ammontare del Cambiamento di un Numero

Ecco un esempio pratico di calcolo dell'ammontare del cambiamento (differenza) in uno stream di valori numerici.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (numeri al quadrato)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (differenza: +${result.diff})`);
});

// Output:
// 0 → 1 (differenza: +1)
// 1 → 4 (differenza: +3)
// 4 → 9 (differenza: +5)
// 9 → 16 (differenza: +7)
// 16 → 25 (differenza: +9)
```

## 🎯 Determinazione della Direzione dello Scroll

Il seguente è un esempio di determinazione della direzione dello scroll (su/giù).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Crea area di output con visualizzazione fissa
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Contenuto fittizio per lo scrolling
const content = document.createElement('div');
content.style.height = '200vh'; // Altezza pagina doppia
content.innerHTML = '<h1>Scorri verso il basso</h1>';
document.body.appendChild(content);

// Ottieni posizione scroll
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // Throttle ogni 100ms
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ Giù' : '↑ Su';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Direzione scroll: ${direction}<br>
    Posizione precedente: ${prevY.toFixed(0)}px<br>
    Posizione corrente: ${currY.toFixed(0)}px<br>
    Movimento: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- Man mano che la pagina viene scrollata, le informazioni su direzione e posizione vengono visualizzate in un'area fissa in alto a destra.
- `pairwise` ti permette di ottenere automaticamente la posizione di scroll precedente e corrente in coppia.

## 🎯 Utilizzo di pairwise Type-Safe

Questo è un esempio di utilizzo dell'inferenza dei tipi di TypeScript.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Output:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 Confronto con bufferCount(2, 1)

`pairwise()` è equivalente a `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Output: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Output: [1,2], [2,3], [3,4], [4,5]
```

**Differenze di Utilizzo**:
- `pairwise()`: Tratta esplicitamente coppie di due valori consecutivi, e l'intento del codice è chiaro
- `bufferCount(2, 1)`: Più flessibile (può gestire più di 3 dimensioni di finestra)

## ⚠️ Note

### Il Primo Valore Non Viene Emesso

Poiché `pairwise` non emette nulla fino a quando i due valori non sono allineati, il primo valore non può essere ottenuto da solo.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Completato')
});

// Output:
// Completato
// (Nessun valore viene emesso)
```

**Contromisura**: Se vuoi elaborare anche il primo valore, aggiungi un valore iniziale con `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Output:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Utilizzo Memoria

Poiché `pairwise` mantiene sempre solo un valore precedente, è efficiente in termini di memoria.

## 📚 Operatori Correlati

- [`scan`](/it/guide/operators/transformation/scan) - Processo di accumulo più complesso
- [`bufferCount`](/it/guide/operators/transformation/bufferCount) - Riassumi valori per ogni numero specificato di elementi
- [`distinctUntilChanged`](/it/guide/operators/filtering/distinctUntilChanged) - Rimuovi valori duplicati consecutivi
- [`startWith`](/it/guide/operators/utility/startWith) - Aggiungi valore iniziale

## Riepilogo

L'operatore `pairwise` emette due valori consecutivi come coppie `[valore precedente, valore corrente]`. Questo è molto utile per **situazioni dove serve un confronto tra il valore precedente e quello corrente**, come tracciare i movimenti del mouse, calcolare i cambiamenti di prezzo e rilevare transizioni di stato. Nota che il primo valore non viene emesso fino all'arrivo del secondo valore, ma questo può essere gestito aggiungendo un valore iniziale con `startWith`.
