---
description: concatWith è un operatore di combinazione RxJS che unisce altri Observable in sequenza dopo il completamento dell'Observable originale. È ideale per elaborazione sequenziale all'interno di una pipeline, elaborazione successiva dopo il completamento, caricamento dati a stadi, e altre situazioni in cui vuoi aggiungere elaborazione successiva come estensione dello stream principale. La versione pipeable operator è conveniente per l'uso all'interno di una pipeline.
titleTemplate: ':title'
---

# concatWith - Concat Sequenziale

L'operatore `concatWith` **concatena sequenzialmente** gli altri Observable specificati dopo che l'Observable originale `completa`.
Questa è la versione Pipeable Operator della Funzione di Creazione `concat`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Output: A → B → C → D → E → F
```

- Dopo che `obs1$` completa, `obs2$` inizia, e dopo che `obs2$` completa, `obs3$` inizia.
- Può essere usato all'interno delle catene `.pipe()`, rendendo facile la combinazione con altri operatori.

[🌐 Documentazione Ufficiale RxJS - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 Pattern di Utilizzo Tipici

- **Elaborazione sequenziale all'interno di una pipeline**: Combina dati aggiuntivi in sequenza allo stream trasformato
- **Elaborazione successiva dopo il completamento**: Aggiungi pulizia e notifiche dopo che l'elaborazione principale completa
- **Caricamento dati a stadi**: Acquisisci dati aggiuntivi sequenzialmente dopo l'acquisizione dati iniziale


## 🧠 Esempio di Codice Pratico (con UI)

Esempio di visualizzazione di elementi raccomandati correlati in ordine dopo la visualizzazione dei risultati di ricerca principali.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Crea area output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio Pratico concatWith:</h3>';
document.body.appendChild(output);

// Risultati di ricerca principali
const searchResults$ = of('🔍 Risultato Ricerca 1', '🔍 Risultato Ricerca 2', '🔍 Risultato Ricerca 3').pipe(
  delay(500)
);

// Elementi raccomandati 1
const recommendations1$ = of('💡 Prodotto Raccomandato A', '💡 Prodotto Raccomandato B').pipe(
  delay(300)
);

// Elementi raccomandati 2
const recommendations2$ = of('⭐ Prodotto Popolare X', '⭐ Prodotto Popolare Y').pipe(
  delay(300)
);

// Combina in sequenza e visualizza
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- I risultati di ricerca vengono visualizzati per primi,
- Poi i prodotti raccomandati vengono visualizzati in ordine.
- Può essere usato in combinazione con altri operatori come `map` all'interno della pipeline.


## 🔄 Differenza dalla Funzione di Creazione `concat`

### Differenze Base

| | `concat` (Funzione di Creazione) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **Posizione di Utilizzo** | Usato come funzione indipendente | Usato all'interno della catena `.pipe()` |
| **Sintassi** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Primo Stream** | Tratta tutti ugualmente | Tratta come stream principale |
| **Vantaggio** | Semplice e leggibile | Facile da combinare con altri operatori |

### Esempi di Utilizzo Specifici

**La Funzione di Creazione è Raccomandata per Sola Combinazione Semplice**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Semplice e leggibile
concat(part1$, part2$, part3$).subscribe(console.log);
// Output: A → B → C → D → E → F
```

**Il Pipeable Operator è Raccomandato se Serve Trasformazione**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Versione Funzione di Creazione - diventa prolissa
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Versione Pipeable Operator - completato in una pipeline
userData$
  .pipe(
    filter(user => user.age >= 30),  // Solo 30 anni o più
    map(user => user.name),          // Estrai solo nome
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Output: Alice → Charlie
```

**Quando si Aggiunge Elaborazione Successiva allo Stream Principale**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, mapTo } from 'rxjs'; } from 'rxjs';

// Crea pulsante e area output
const button = document.createElement('button');
button.textContent = 'Clicca 3 volte';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Versione Pipeable Operator - naturale come estensione dello stream principale
clicks$
  .pipe(
    take(3),                          // Ottieni i primi 3 click
    map(() => 'Cliccato'),
    concatWith(of('Completato'))       // Aggiungi messaggio dopo il completamento
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Scrivere lo stesso comportamento con versione Funzione di Creazione...
// ❌ Versione Funzione di Creazione - serve separare lo stream principale
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    map(() => 'Cliccato')
  ),
  of('Completato')
).subscribe(console.log);
```

### Riepilogo

- **`concat`**: Ottimale per combinare semplicemente più stream
- **`concatWith`**: Ottimale quando vuoi aggiungere elaborazione successiva allo stream principale mentre lo trasformi o elabori


## ⚠️ Note Importanti

### Ritardo Dovuto all'Attesa del Completamento

L'Observable successivo non inizierà fino a quando l'Observable originale non completa.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // Completa con 3 valori
  concatWith(of('Completo'))
).subscribe(console.log);
// Output: 0 → 1 → 2 → Completo
```

### Gestione Errori

Se si verifica un errore nell'Observable precedente, gli Observable successivi non verranno eseguiti.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Si è verificato un errore'))
  .pipe(
    catchError((err: unknown) => of('Errore recuperato')),
    concatWith(of('Processo successivo'))
  )
  .subscribe(console.log);
// Output: Errore recuperato → Processo successivo
```


## 📚 Operatori Correlati

- **[concat](/it/guide/creation-functions/combination/concat)** - Versione Funzione di Creazione
- **[mergeWith](/it/guide/operators/combination/mergeWith)** - Versione Pipeable per combinazione parallela
- **[concatMap](/it/guide/operators/transformation/concatMap)** - Mappa ogni valore sequenzialmente
