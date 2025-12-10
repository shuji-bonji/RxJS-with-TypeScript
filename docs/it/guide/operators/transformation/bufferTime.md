---
description: "L'operatore bufferTime raccoglie valori a intervalli di tempo regolari per l'elaborazione batch: Perfetto per aggregare eventi, log e stream di dati real-time"
titleTemplate: ':title | RxJS'
---

# bufferTime - Emetti Valori Raccolti a Intervalli Regolari

L'operatore `bufferTime` emette **un array di valori** a intervalli di tempo specificati.
Questo è utile quando vuoi separare lo stream per una certa quantità di tempo e trattarlo come un processo batch.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Emetti valori ogni 100ms
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('Valori raccolti in 1 secondo:', buffer);
});

// Esempio di output:
// Valori raccolti in 1 secondo: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valori raccolti in 1 secondo: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- I valori emessi in un secondo vengono raggruppati in un array ed emessi in sequenza.

[🌐 Documentazione Ufficiale RxJS - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Pattern di Utilizzo Tipici

- Invio batch a intervalli regolari
- Elaborare operazioni utente in batch (es. operazioni di drag)
- Raccogliere dati da sensori e dispositivi IoT
- Riduzione e compressione di informazioni di log e trace

## 🧠 Esempio di Codice Pratico (con UI)

Bufferizza gli eventi click per 1 secondo ed emettili insieme ogni secondo.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream eventi click
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Click in 1 secondo: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- Il numero di click al secondo viene visualizzato come riepilogo.
- Il processo di buffering ti permette di gestire insieme le occorrenze successive di eventi.
