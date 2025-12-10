---
description: bufferCount è un operatore di conversione RxJS che emette un array di valori per ogni numero specificato di elementi. È ideale per elaborazione batch, aggregazione dati per conteggio fisso, suddivisione pacchetti e altro controllo di stream basato sul conteggio, e abilita operazioni su array type-safe attraverso l'inferenza dei tipi di TypeScript.
---

# bufferCount - Raccogli Valori per Conteggio Specificato

L'operatore `bufferCount` **raggruppa insieme** un numero specificato di valori emessi e li emette come array.
Questo è utile per l'elaborazione batch dove vuoi separare i valori per conteggio.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Emetti valori ogni 100ms
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Valori ogni 5:', buffer);
});

// Output:
// Valori ogni 5: [0, 1, 2, 3, 4]
// Valori ogni 5: [5, 6, 7, 8, 9]
// ...
```

- Emette un array di 5 valori alla volta.
- È unico in quanto raggruppa su base di **conteggio**, non su base temporale.

[🌐 Documentazione Ufficiale RxJS - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Pattern di Utilizzo Tipici

- Suddivisione e invio pacchetti dati
- Salvataggio batch o elaborazione batch per conteggio specifico
- Aggregazione di eventi di input per un certo numero di occorrenze

## 🧠 Esempio di Codice Pratico (con UI)

Questo è un esempio di visualizzazione di un riepilogo delle pressioni di tasti ogni 5 pressioni.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream eventi input da tastiera
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 input: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Ogni volta che un tasto viene premuto cinque volte, quelle cinque pressioni vengono visualizzate insieme.
- Puoi sperimentare il processo di aggregazione in base al conteggio.
