---
description: "La Funzione di Creazione merge sottoscrive più Observable simultaneamente e unisce i valori in tempo reale: Essenziale per integrare stream di eventi paralleli"
---

# merge - unisci più stream simultaneamente

`merge` è una Funzione di Creazione che sottoscrive più Observable simultaneamente,
e emette i valori non appena vengono emessi da ogni Observable.

## Sintassi base e utilizzo

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Esempio output:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Sottoscrive tutti gli Observable simultaneamente, e i valori fluiranno in **ordine di emissione**.
- Non c'è garanzia di ordine, e **dipende** dal timing di emissione di ogni Observable.


[🌐 Documentazione Ufficiale RxJS - `merge`](https://rxjs.dev/api/index/function/merge)

## Pattern di utilizzo tipici

- **Unisci** eventi asincroni multipli (es., input utente e notifiche backend)
- **Aggrega stream di dati multipli in un singolo stream**.
- **Combina sorgenti di informazioni parallele, es., aggiornamenti in tempo reale e integrazione polling**.

## Esempi di codice pratici (con UI)

Combina eventi click e timer in tempo reale.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico merge:</h3>';
document.body.appendChild(output);

// Crea elemento pulsante
const button = document.createElement('button');
button.textContent = 'Clicca per attivare evento';
document.body.appendChild(button);

// Stream click
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Click pulsante rilevato')
);

// Stream timer
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Evento timer (${val})`)
);

// merge e visualizza
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Clicca su un pulsante e un evento viene generato** immediatamente,
- **Il timer attiva un evento ripetuto** ogni 3 secondi.
- Puoi sperimentare come due diversi tipi di Observable possono essere uniti in **tempo reale**.


## Operatori Correlati

- **[mergeWith](/it/guide/operators/combination/mergeWith)** - Versione Pipeable Operator (usata in pipeline)
- **[mergeMap](/it/guide/operators/transformation/mergeMap)** - mappa e concatena ogni valore in parallelo
- **[concat](/it/guide/creation-functions/combination/concat)** - Funzione di Creazione per concatenazione sequenziale
