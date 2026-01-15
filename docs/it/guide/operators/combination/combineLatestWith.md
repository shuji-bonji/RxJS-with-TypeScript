---
description: combineLatestWith è un operatore di combinazione RxJS che combina l'Observable originale con gli ultimi valori di altri Observable per formare un nuovo stream. È la versione Pipeable Operator della Funzione di Creazione combineLatest, ed è ideale per validazione form in tempo reale, combinazione di dati da più sensori, combinazione di filtri di ricerca, e altre situazioni in cui vuoi integrare gli ultimi valori di altri stream mentre trasformi o elabori lo stream principale.
titleTemplate: ':title | RxJS'
---

# combineLatestWith - Combina ultimi

L'operatore `combineLatestWith` combina gli **ultimi valori** dell'Observable originale e di altri Observable specificati in un nuovo stream.
Questa è la versione Pipeable Operator della Funzione di Creazione `combineLatest`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map } from 'rxjs';

const source1$ = interval(1000); // 0, 1, 2, ...
const source2$ = interval(1500); // 0, 1, 2, ...

source1$
  .pipe(
    combineLatestWith(source2$),
    map(([val1, val2]) => `Stream1: ${val1}, Stream2: ${val2}`)
  )
  .subscribe(console.log);

// Esempio di output:
// Stream1: 0, Stream2: 0
// Stream1: 1, Stream2: 0
// Stream1: 2, Stream2: 0
// Stream1: 2, Stream2: 1
// Stream1: 3, Stream2: 1
// ...
```

- Attendi fino a quando tutti gli stream hanno emesso **almeno una volta**, poi emetti la combinazione degli ultimi valori **ogni volta che uno di essi emette**.
- Poiché è accettato in forma tupla, è type-safe in TypeScript.

[🌐 Documentazione Ufficiale RxJS - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)


## 💡 Pattern di Utilizzo Tipici

- **Validazione form in tempo reale**: Combina e valida gli ultimi valori di più campi
- **Integrazione di più sensori**: Visualizzazione simultanea di dati a frequenze diverse come temperatura, umidità, ecc.
- **Filtri di ricerca combinati**: Integra selezione categoria e inserimento parole chiave
- **Anteprima live**: Anteprima in tempo reale combinando più valori di configurazione


## 🧠 Esempio di Codice Pratico (con UI)

Esempio di cambio colore (RGB) in tempo reale con più slider.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, combineLatestWith } from 'rxjs';

// Costruisci la UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>Esempio Pratico combineLatestWith: Selettore Colore RGB</h3>
  <div>
    <label>Rosso: <input type="range" id="red" min="0" max="255" value="128"></label>
    <span id="red-value">128</span>
  </div>
  <div>
    <label>Verde: <input type="range" id="green" min="0" max="255" value="128"></label>
    <span id="green-value">128</span>
  </div>
  <div>
    <label>Blu: <input type="range" id="blue" min="0" max="255" value="128"></label>
    <span id="blue-value">128</span>
  </div>
  <div id="preview" style="width: 200px; height: 100px; border: 1px solid #ccc; margin-top: 10px;"></div>
`;
document.body.appendChild(container);

// Ottieni elementi slider
const redSlider = document.getElementById('red') as HTMLInputElement;
const greenSlider = document.getElementById('green') as HTMLInputElement;
const blueSlider = document.getElementById('blue') as HTMLInputElement;

const redValue = document.getElementById('red-value')!;
const greenValue = document.getElementById('green-value')!;
const blueValue = document.getElementById('blue-value')!;
const preview = document.getElementById('preview')!;

// Stream per ogni slider
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

const green$ = fromEvent(greenSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

const blue$ = fromEvent(blueSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(128)
);

// ✅ Versione Pipeable Operator - integra altri nello stream principale
red$
  .pipe(
    combineLatestWith(green$, blue$)
  )
  .subscribe(([r, g, b]) => {
    // Aggiorna visualizzazione valori
    redValue.textContent = String(r);
    greenValue.textContent = String(g);
    blueValue.textContent = String(b);

    // Aggiorna colore di sfondo anteprima
    preview.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
  });
```

- Muovendo qualsiasi slider si aggiornerà **immediatamente** l'anteprima con gli ultimi valori RGB combinati.
- Dopo che tutti gli slider sono stati manipolati almeno una volta, l'ultima combinazione viene sempre riflessa.


## 🔄 Differenza dalla Funzione di Creazione `combineLatest`

### Differenze Base

| | `combineLatest` (Funzione di Creazione) | `combineLatestWith` (Pipeable Operator) |
|:---|:---|:---|
| **Posizione di Utilizzo** | Usato come funzione indipendente | Usato all'interno della catena `.pipe()` |
| **Sintassi** | `combineLatest([obs1$, obs2$, obs3$])` | `obs1$.pipe(combineLatestWith(obs2$, obs3$))` |
| **Primo Stream** | Tratta tutti ugualmente | Tratta come stream principale |
| **Vantaggio** | Semplice e leggibile | Facile da combinare con altri operatori |

### Esempi di Utilizzo Specifici

**La Funzione di Creazione è Raccomandata per Sola Combinazione Semplice**

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const width$ = fromEvent(window, 'resize').pipe(map(() => window.innerWidth));
const height$ = fromEvent(window, 'resize').pipe(map(() => window.innerHeight));

// Semplice e leggibile
combineLatest([width$, height$]).subscribe(([w, h]) => {
  console.log(`Dimensione finestra: ${w} x ${h}`);
});
```

**Il Pipeable Operator è Raccomandato Quando si Aggiunge Elaborazione di Trasformazione allo Stream Principale**

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, throttleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

// ✅ Versione Pipeable Operator - completato in una pipeline
clicks$
  .pipe(
    throttleTime(500),           // Previeni click rapidi
    map(() => Date.now()),       // Converti in timestamp
    startWith(0),                // Imposta valore iniziale
    combineLatestWith(timer$),   // Integra con timer
    map(([clickTime, tick]) => ({
      lastClick: clickTime,
      elapsed: tick
    }))
  )
  .subscribe(data => {
    console.log(`Ultimo click: ${data.lastClick}, Trascorso: ${data.elapsed} secondi`);
  });

// ❌ Versione Funzione di Creazione - diventa prolissa
import { combineLatest } from 'rxjs';
combineLatest([
  clicks$.pipe(
    throttleTime(500),
    map(() => Date.now()),
    startWith(0)
  ),
  timer$
]).pipe(
  map(([clickTime, tick]) => ({
    lastClick: clickTime,
    elapsed: tick
  }))
).subscribe(data => {
  console.log(`Ultimo click: ${data.lastClick}, Trascorso: ${data.elapsed} secondi`);
});
```

### Riepilogo

- **`combineLatest`**: Ottimale per combinare semplicemente più stream
- **`combineLatestWith`**: Ottimale quando vuoi integrare altri stream mentre trasformi o elabori lo stream principale


## ⚠️ Note Importanti

### Attendi Fino a Quando Tutti gli Stream Emettono Almeno Una Volta

I valori non verranno emessi fino a quando tutti gli Observable non hanno emesso almeno una volta.

```ts
import { of, timer } from 'rxjs';
import { combineLatestWith } from 'rxjs';

of(1, 2, 3).pipe(
  combineLatestWith(
    timer(1000),  // Emette dopo 1 secondo
  )
).subscribe(console.log);
// Output: [3, 0]
// * Attende fino a quando timer$ emette, poi combina con l'ultimo valore (3) di of() in quel momento
```

### Attenzione agli Aggiornamenti ad Alta Frequenza

Se uno degli stream viene aggiornato frequentemente, il risultato combinato verrà emesso frequentemente di conseguenza.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(100).pipe(
  take(5),
  combineLatestWith(interval(1000).pipe(take(3)))
).subscribe(console.log);
// Output:
// [0, 0]
// [1, 0]
// [2, 0]
// [3, 0]
// [4, 0]
// [4, 1]
// [4, 2]
```

Controlla la frequenza di aggiornamento con `throttleTime` o `debounceTime` se necessario.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, throttleTime, map } from 'rxjs';

const mouseMoves$ = fromEvent(document, 'mousemove').pipe(
  throttleTime(100),  // Limita ogni 100ms
  map(e => ({ x: (e as MouseEvent).clientX, y: (e as MouseEvent).clientY }))
);

const timer$ = interval(1000);

mouseMoves$
  .pipe(combineLatestWith(timer$))
  .subscribe(([pos, tick]) => {
    console.log(`Posizione: (${pos.x}, ${pos.y}), Tick: ${tick}`);
  });
```

### Gestione Errori

Se si verifica un errore in qualsiasi Observable, l'intero stream termina con un errore.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
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

- **[combineLatest](/it/guide/creation-functions/combination/combineLatest)** - Versione Funzione di Creazione
- **[zipWith](/it/guide/operators/combination/zipWith)** - Accoppia valori corrispondenti (ordine garantito)
- **[withLatestFrom](/it/guide/operators/combination/withLatestFrom)** - Combina solo quando lo stream principale emette
