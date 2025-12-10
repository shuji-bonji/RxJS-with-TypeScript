---
description: "L'operatore scan è un operatore RxJS che accumula ciascun valore in modo sequenziale ed emette risultati intermedi. A differenza di reduce(), emette risultati ogni volta che arriva un valore, quindi viene utilizzato per aggregazioni in tempo reale, gestione stato, contatori cumulativi e calcoli in streaming. Spiega l'implementazione type-safe con TypeScript."
---

# scan - Genera valori in modo cumulativo

L'operatore `scan` applica una funzione cumulativa a ciascun valore del flusso ed emette **risultati intermedi sequenziali**.
È simile a `Array.prototype.reduce` degli array, ma differisce nel fatto che emette sequenzialmente risultati intermedi prima che tutti i valori siano arrivati.

## 🔰 Sintassi di Base e Utilizzo

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Output: 1, 3, 6, 10, 15

```

- `acc` è il valore cumulativo, `curr` è il valore corrente.
- Inizia dal valore iniziale (in questo caso `0`) e accumula sequenzialmente.

[🌐 Documentazione Ufficiale RxJS - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 Pattern di Utilizzo Tipici

- Conteggio incrementale o aggregazione punteggi
- Gestione stato di validazione form in tempo reale
- Elaborazione cumulativa di eventi bufferizzati
- Costruzione dati per grafici di aggregazione in tempo reale

## 🧠 Esempio di Codice Pratico (con UI)

Visualizza il conteggio cumulativo dei clic ogni volta che si clicca il pulsante.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Creazione pulsante
const button = document.createElement('button');
button.textContent = 'Clicca';
document.body.appendChild(button);

// Creazione area di output
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Accumula eventi di clic
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Numero di clic: ${count}`;
  });
```

- Ogni volta che si clicca il pulsante, il contatore aumenta di 1.
- Usando `scan`, è possibile scrivere **logica di conteggio semplice senza gestione stato**.
