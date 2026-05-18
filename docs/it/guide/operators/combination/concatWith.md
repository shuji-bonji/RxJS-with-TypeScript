---
description: "ConcatWith è un operatore di join di RxJS che unisce altri Observable in sequenza dopo il completamento dell'Observable originale. È ideale per le situazioni in cui si desidera aggiungere un'elaborazione successiva come estensione del flusso principale, come ad esempio l'elaborazione sequenziale in una pipeline, l'elaborazione successiva al completamento, il caricamento di dati a tappe, ecc."
---

# concatWith - concatena i flussi in sequenza

L'operatore `concatWith` **concatena** in sequenza gli altri Observable specificati dopo che l'Observable originale è stato completato.
È la versione Pipeable Operator di `concat` di Creation Function.

## 🔰 Sintassi e utilizzo di base

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Uscita: A → B → C → D → E → F
```

- `obs1$` completa prima che inizi `obs2$`, `obs2$` completa prima che inizi `obs3$`.
- Può essere usato all'interno di una catena di `.pipe()`, rendendo più facile la combinazione con altri operatori.

[🌐 Documentazione ufficiale di RxJS - `concatWith`](https://rxjs.dev/api/operators/concatWith)

## 💡 Tipico modello di utilizzo.

- **Elaborazione sequenziale nella pipeline**: i dati aggiuntivi vengono concatenati in sequenza nello stream convertito.
- **Elaborazione successiva al completamento**: aggiunta di pulizia e notifiche al termine dell'elaborazione principale.
- **Caricamento dei dati in sequenza**: acquisizione sequenziale di dati aggiuntivi dopo l'acquisizione iniziale dei dati.

## 🧠 Esempi pratici di codice (con interfaccia utente)

Esempio di visualizzazione dei risultati della ricerca principale, seguiti da una sequenza di elementi consigliati correlati.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Creazione di un'area di output
const output = document.createElement('div');
output.innerHTML = '<h3>concatWith Esempi pratici di:</h3>';
document.body.appendChild(output);

// Risultati principali della ricerca
const searchResults$ = of('🔍 Risultati della ricerca1', '🔍 Risultati della ricerca2', '🔍 Risultati della ricerca3').pipe(
  delay(500)
);

// Articoli consigliati1
const recommendations1$ = of('💡 Articoli consigliatiA', '💡 Articoli consigliatiB').pipe(
  delay(300)
);

// Articoli consigliati2
const recommendations2$ = of('⭐ Articoli popolariX', '⭐ Articoli popolariY').pipe(
  delay(300)
);

// Combinati e visualizzati in ordine
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

- I risultati della ricerca vengono visualizzati per primi,
- gli elementi consigliati vengono poi visualizzati in sequenza.
- Può essere usato in combinazione con altri operatori, come map, nella pipeline.

## 🔄 Differenze con la Creation Function `concat`.

### Differenze di base.

|  | concat (Creation Function) | concatWith` (Pipeable Operator) |
|---|---|---|
| **Dove si usa** | Usata come funzione indipendente | Usata nella catena `.pipe()`. |
| **Come scrivere** | concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Primo flusso**. | Tratta tutti come uguali | Tratta come flusso principale |
| **Vantaggi**. | Semplice e di facile lettura | Facile da combinare con altri operatori |

### Esempi specifici di utilizzo

**Se si desidera solo una semplice unione, la Creation Function è la soluzione migliore**.

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Semplice e di facile lettura
concat(part1$, part2$, part3$).subscribe(console.log);
// Uscita: A → B → C → D → E → F
```

**Se è necessario convertire nel mezzo, si consiglia Pipeable Operator**.

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Creation FunctionEdizione - Ridondante
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Pipeable OperatorEdizione - Completo in un'unica linea
userData$
  .pipe(
    filter(user => user.age >= 30),  // 30Solo per chi ha più di 18 anni
    map(user => user.name),          // Estrazione dei soli nomi
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Uscita: Alice → Charlie
```

**Se si desidera aggiungere un'elaborazione successiva al flusso principale**.

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, map } from 'rxjs';

// Creare un pulsante e un'area di output
const button = document.createElement('button');
button.textContent = '3Fare clic una volta';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Pipeable OperatorEdizione - Naturale come estensione del flusso principale
clicks$
  .pipe(
    take(3),                          // Primo3Ottenere i clic
    map(() => 'Cliccato'),
    concatWith(of('Completato'))    // Messaggio aggiuntivo dopo il completamento
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Lo stesso comportamento può essereCreation FunctionSe scritto in un'edizione...
// ❌ Creation FunctionEdizione - Il mainstream deve essere scritto separatamente
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    map(() => 'Cliccato')
  ),
  of('Completato')
).subscribe(console.log);
```

### Riepilogo.

- concat`**: ideale per unire più flussi in modo semplice
- **`concatWith`**: ideale se si vogliono aggiungere flussi successivi mentre si trasforma o si elabora il flusso principale

## ⚠️ Note.

### Ritardi causati dall'attesa di completamento

L'Observable successivo non si avvia finché l'Observable originale non è stato completato.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // 3Completato in una
  concatWith(of('Completato'))
).subscribe(console.log);
// Uscita: 0 → 1 → 2 → Completato
```

### Gestione degli errori.

Se si verifica un errore nell'Observable precedente, l'Observable successivo non verrà eseguito.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Si è verificato un errore'))
  .pipe(
    catchError((err: unknown) => of('Recupero dall'errore')),
    concatWith(of('Processo successivo'))
  )
  .subscribe(console.log);
// Uscita: Recupero dall'errore → Processo successivo
```

## 📚 Operatori correlati.

- **[concat](/it/guide/creation-functions/combinazione/concat)** - Versione di Creation Function
- **[mergeWith](/it/guide/operators/combinazione/mergeWith)** - Versione per unire in parallelo
- **[concatMap](/it/guide/operators/trasformazione/concatMap)** - mappatura sequenziale di singoli valori
