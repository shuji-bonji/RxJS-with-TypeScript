---
description: La Funzione di Creazione zip allinea e accoppia i valori nell'ordine corrispondente da più Observable e li emette contemporaneamente quando tutte le sorgenti hanno emesso i loro valori uno per uno.
---

# zip - accoppia valori corrispondenti

`zip` è una Funzione di Creazione che raggruppa insieme i **valori ordinati corrispondenti** emessi da più Observable e li emette come array o tupla.
Aspetta che i valori arrivino da tutti gli Observable sorgente, uno alla volta, e crea coppie quando sono pronti.


## Sintassi base e utilizzo

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// Output:
// A 0
// B 10
// C 20
```

- Quando ogni Observable emette un valore alla volta, viene creata e emessa una coppia.
- Se uno è in ritardo, aspetterà finché entrambi non sono allineati.

[🌐 Documentazione Ufficiale RxJS - `zip`](https://rxjs.dev/api/index/function/zip)


## Pattern di utilizzo tipici

- **Mappare richieste a risposte**
- **Accoppiare sincronicamente ID con dati corrispondenti**
- **Combinare stream multipli elaborati in parallelo in un singolo set**


## Esempi di codice pratici (con UI)

Esempio di **combinazione e visualizzazione** di sorgenti dati diverse (frutta e prezzo).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico zip:</h3>';
document.body.appendChild(output);

// Stream nomi frutta
const fruits$ = of('🍎 Mela', '🍌 Banana', '🍇 Uva');

// Stream prezzi (emesso ogni 2 secondi)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zip e visualizza
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - €${price}`;
  output.appendChild(item);
});
```

- Le liste di frutta e prezzi vengono accoppiate e visualizzate **quando** sono allineate in corrispondenza uno a uno.
- Se uno manca, non verrà emesso a quel punto.


## Operatori Correlati

- **[zipWith](/it/guide/operators/combination/zipWith)** - Versione Pipeable Operator (usata in pipeline)
- **[combineLatest](/it/guide/creation-functions/combination/combineLatest)** - Funzione di Creazione per combinare ultimi valori
- **[withLatestFrom](/it/guide/operators/combination/withLatestFrom)** - solo lo stream principale attiva
