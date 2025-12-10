---
description: buffer è un operatore RxJS che emette un array di valori accumulati quando un altro Observable emette un valore, rendendolo ideale per l'elaborazione batch guidata da eventi.
titleTemplate: ':title | RxJS'
---

# buffer - Raccogli Valori al Timing di un Altro Observable

L'operatore `buffer` accumula i valori di un Observable sorgente **fino a quando** un altro Observable emette un valore, e poi emette i valori accumulati come un **array** in quel momento.
Questo è utile quando vuoi controllare il buffering in base a eventi o segnali esterni, piuttosto che per tempo o numero di elementi.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Emetti valori ogni 100ms
const source$ = interval(100);

// Usa l'evento click come trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Valori accumulati fino al click:', bufferedValues);
});

// Esempio di output (emette ad ogni click):
// Valori accumulati fino al click: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Valori accumulati fino al click: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Ogni volta che `clicks$` emette un valore, i valori accumulati fino a quel punto vengono emessi come array.
- La caratteristica è che la delimitazione del buffer può essere controllata da un Observable esterno.

[🌐 Documentazione Ufficiale RxJS - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Pattern di Utilizzo Tipici

- Elaborazione batch attivata da azioni utente
- Raccolta e trasmissione dati basata su segnali esterni
- Raggruppamento eventi con delimitazione dinamica
- Invio batch quando viene stabilita una connessione WebSocket o API

## 🔍 Differenza da bufferTime / bufferCount

| Operatore | Timing del Delimitatore | Utilizzo |
|:---|:---|:---|
| `buffer` | **Un altro Observable emette** | Controllo guidato da eventi |
| `bufferTime` | **Intervallo di tempo fisso** | Elaborazione batch basata sul tempo |
| `bufferCount` | **Conteggio fisso** | Elaborazione batch basata sul conteggio |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Trigger ogni 1 secondo
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Valori ogni secondo:', values);
});

// Output:
// Valori ogni secondo: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valori ogni secondo: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Esempio di Codice Pratico (con UI)

Questo è un esempio di attivazione del click di un bottone e registrazione di tutti gli eventi di movimento del mouse fino a quel punto insieme.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Crea bottone e area di output
const button = document.createElement('button');
button.textContent = 'Registra Movimento Mouse';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento movimento mouse
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Trigger al click del bottone
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Eventi rilevati: ${positions.length} elementi`;
  console.log(message);
  console.log('Dati coordinate:', positions.slice(0, 5)); // Mostra solo i primi 5
  output.textContent = message;
});
```

- Tutti i movimenti del mouse fino al click del bottone vengono memorizzati in un buffer.
- Poiché gli eventi vengono elaborati insieme al momento del click, è possibile l'elaborazione batch a timing arbitrario.

## 🎯 Esempio Avanzato con Trigger Multipli

È possibile un controllo più flessibile combinando più Observable trigger.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// Trigger multipli: click o 5 secondi trascorsi
const clicks$ = fromEvent(document, 'click').pipe(mapTo('click'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Output buffer (${values.length} elementi):`, values);
});
```

## ⚠️ Note

### Attenzione ai Memory Leak

Poiché `buffer` continua ad accumulare valori fino al prossimo trigger, potrebbe consumare memoria eccessiva se un trigger non si verifica per molto tempo.

```ts
// Esempio sbagliato: Il trigger potrebbe non verificarsi
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // Il trigger non si verifica mai, il buffer accumula all'infinito
).subscribe();
```

**Contromisure**:
- Limita la dimensione massima del buffer in combinazione con `bufferTime` e `bufferCount`
- Aggiungi la gestione del timeout

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Trigger multipli: click o 5 secondi trascorsi
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout dopo massimo 10 secondi

source$.pipe(
  buffer(race(clicks$, timeout$)) // Emetti al primo che arriva
).subscribe(values => {
  console.log('Buffer:', values);
});
```

## 📚 Operatori Correlati

- [`bufferTime`](/it/guide/operators/transformation/bufferTime) - Buffering basato sul tempo
- [`bufferCount`](/it/guide/operators/transformation/bufferCount) - Buffering basato sul conteggio
- [`bufferToggle`](/it/guide/operators/transformation/bufferToggle) - Controllo buffering con Observable di inizio e fine
- [`bufferWhen`](/it/guide/operators/transformation/bufferWhen) - Buffering con condizioni di chiusura dinamiche
- [`window`](/it/guide/operators/transformation/windowTime) - Restituisce Observable invece del buffer

## Riepilogo

L'operatore `buffer` è uno strumento potente per elaborare un batch di valori attivato da un Observable esterno. Permette l'elaborazione batch **guidata da eventi**, piuttosto che per tempo o numero di elementi. Tuttavia, fai attenzione ai memory leak quando i trigger non si verificano.
