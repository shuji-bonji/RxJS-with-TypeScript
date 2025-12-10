---
description: "Il Pipeable Operator mergeWith sottoscrive più Observable simultaneamente e li unisce in parallelo: Ideale per integrare più sorgenti di eventi"
titleTemplate: ':title'
---

# mergeWith - Unisci Più Stream Simultaneamente All'interno di una Pipeline

L'operatore `mergeWith` **sottoscrive simultaneamente** l'Observable originale e gli altri Observable specificati,
e unisce i valori emessi da ciascuno in tempo reale.
Questa è la versione Pipeable Operator della Funzione di Creazione `merge`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// Esempio di output:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Tutti gli Observable vengono sottoscritti simultaneamente, e i valori fluiscono **nell'ordine in cui vengono emessi**.
- Non c'è garanzia di ordine, e **dipende dal timing di emissione di ogni Observable**.

[🌐 Documentazione Ufficiale RxJS - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 Pattern di Utilizzo Tipici

- **Integrare più sorgenti di eventi**: Combina operazioni utente e aggiornamenti automatici
- **Unire fetch di dati paralleli**: Aggrega risposte da più API in un singolo stream
- **Unire aggiornamenti real-time**: Integra WebSocket e polling


## 🧠 Esempio di Codice Pratico (con UI)

Esempio di integrazione di eventi click utente e timer di aggiornamento automatico per visualizzare notifiche.

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// Crea area output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio Pratico mergeWith:</h3>';
document.body.appendChild(output);

// Crea pulsante
const button = document.createElement('button');
button.textContent = 'Aggiornamento Manuale';
document.body.appendChild(button);

// Stream click
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 Aggiornamento manuale eseguito')
);

// Timer aggiornamento automatico (ogni 5 secondi)
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 Aggiornamento automatico #${val + 1}`),
  take(3)
);

// Integra entrambi e visualizza
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- Cliccando il pulsante viene visualizzato immediatamente l'aggiornamento manuale,
- Gli aggiornamenti automatici vengono eseguiti anche in parallelo ogni 5 secondi.
- Entrambi gli eventi sono integrati in tempo reale.


## 🔄 Differenza dalla Funzione di Creazione `merge`

### Differenze Base

| | `merge` (Funzione di Creazione) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **Posizione di Utilizzo** | Usato come funzione indipendente | Usato all'interno della catena `.pipe()` |
| **Sintassi** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **Primo Stream** | Tratta tutti ugualmente | Tratta come stream principale |
| **Vantaggio** | Semplice e leggibile | Facile da combinare con altri operatori |

### Esempi di Utilizzo Specifici

**La Funzione di Creazione è Raccomandata per Sola Unione Semplice**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'Click'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'Movimento mouse'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'Pressione tasto'));

// Semplice e leggibile
merge(clicks$, moves$, keypress$).subscribe(console.log);
// Output: Visualizza nell'ordine in cui si verifica qualsiasi evento
```

**Il Pipeable Operator è Raccomandato Quando si Aggiunge Elaborazione di Trasformazione allo Stream Principale**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // Ogni 30 secondi

// ✅ Versione Pipeable Operator - completato in una pipeline
userClicks$
  .pipe(
    throttleTime(1000),           // Previeni click rapidi
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // Solo entro 1 minuto
  )
  .subscribe(event => {
    console.log(`aggiornamento ${event.source}: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });

// ❌ Versione Funzione di Creazione - diventa prolissa
import { merge } from 'rxjs';
merge(
  userClicks$.pipe(
    throttleTime(1000),
    map(() => ({ source: 'user', timestamp: Date.now() }))
  ),
  autoRefresh$.pipe(
    map(() => ({ source: 'auto', timestamp: Date.now() }))
  )
).pipe(
  filter(event => event.timestamp > Date.now() - 60000)
).subscribe(event => {
  console.log(`aggiornamento ${event.source}: ${new Date(event.timestamp).toLocaleTimeString()}`);
});
```

**Quando si Integrano Più Sorgenti Dati**

```ts
import { fromEvent, timer } from 'rxjs';
import { mergeWith, map, startWith } from 'rxjs';

// Crea pulsante
const saveButton = document.createElement('button');
saveButton.textContent = 'Salva';
document.body.appendChild(saveButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream principale: Operazione di salvataggio dell'utente
const manualSave$ = fromEvent(saveButton, 'click').pipe(
  map(() => '💾 Salvataggio manuale')
);

// ✅ Versione Pipeable Operator - aggiungi salvataggio automatico allo stream principale
manualSave$
  .pipe(
    startWith('📝 Modifica iniziata'),
    mergeWith(
      timer(10000, 10000).pipe(map(() => '⏰ Salvataggio automatico'))  // Salvataggio automatico ogni 10 secondi
    )
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    output.appendChild(div);
  });
```

### Riepilogo

- **`merge`**: Ottimale per unire semplicemente più stream su base uguale
- **`mergeWith`**: Ottimale quando vuoi unire altri stream mentre trasformi o elabori lo stream principale


## ⚠️ Note Importanti

### Timing di Completamento

Lo stream unito non completerà fino a quando tutti gli Observable non completano.

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← Aggiungendo questo non completerà mai
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Completo')
});
// Output: 1 → 2 → 3 → 0 → 1 → ✅ Completo
```

### Controllo del Conteggio Esecuzioni Concorrenti

Di default, tutti gli stream vengono eseguiti concorrentemente, ma può essere controllato in combinazione con `mergeMap`.

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // Esegui fino a 2 concorrentemente
  )
).subscribe(console.log);
```

### Gestione Errori

Se si verifica un errore in qualsiasi Observable, l'intero stream termina con un errore.

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('Si è verificato un errore')).pipe(
      catchError(err => of('Errore recuperato'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Output: 0 → Errore recuperato → 1
```


## 📚 Operatori Correlati

- **[merge](/it/guide/creation-functions/combination/merge)** - Versione Funzione di Creazione
- **[concatWith](/it/guide/operators/combination/concatWith)** - Versione Pipeable per combinazione sequenziale
- **[mergeMap](/it/guide/operators/transformation/mergeMap)** - Mappa ogni valore in parallelo
