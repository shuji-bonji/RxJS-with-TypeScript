---
description: "zipWith è un operatore di join di RxJS che allinea e accoppia l'Observable originale con i corrispondenti valori ordinati di altri Observable. È ideale per i join di flussi critici dal punto di vista dell'ordine, come la combinazione dei risultati di un'elaborazione parallela con garanzie di ordine, la mappatura degli ID ai dati, la sincronizzazione di dati correlati pubblicati in momenti diversi, ecc."
---

# zipWith - coppia di valori corrispondenti

L'operatore zipWith raggruppa i **valori ordinati corrispondenti** emessi dall'Observable originale e dagli altri Observable specificati.
Attende che i valori arrivino uno per uno da tutti gli Observable e crea le coppie quando sono pronte.
È la versione di Pipeable Operator di Creation Function, ovvero di zip.

## 🔰 Sintassi e utilizzo di base

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// L'uscita:
// A - 0
// B - 10
// C - 20
// (Dnon viene emesso perché non c'è un valore corrispondente)
```

- Da ogni Observable viene emessa una coppia **quando uno dei valori è completo**.
- Quando un Observable è completo, i valori rimanenti vengono scartati.

[🌐 Documentazione ufficiale di RxJS - `zipWith`](https://rxjs.dev/api/operators/zipWith)

## 💡 Tipico modello di utilizzo.

- Combinare i risultati dell'elaborazione parallela in ordine garantito**: accoppiare i risultati di più chiamate API.
- **Mappatura degli ID ai dati**: combinazione degli ID degli utenti con i dati del profilo corrispondente.
- **Sincronizzazione di flussi**: sincronizzazione di dati correlati emessi in momenti diversi.

## 🧠 Esempi pratici di codice (con UI)

Esempio di abbinamento in sequenza di un elenco di ID utente e dei nomi utente corrispondenti.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Creazione dell'area di output
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Esempi pratici di:</h3>';
document.body.appendChild(output);

// UtenteIDFlusso (pubblicato immediatamente)
const userIds$ = from([101, 102, 103, 104]);

// Flusso di nomi utente (emesso ogni1(emesso ogni secondo)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipe visualizzato
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 UtenteID ${id}: ${name}`;
    output.appendChild(item);
  });

// L'uscita:
// 👤 UtenteID 101: Alice
// 👤 UtenteID 102: Bob
// 👤 UtenteID 103: Carol
// (104non viene visualizzato perché non c'è un nome corrispondente)
```

- Gli ID e i nomi sono accoppiati in una corrispondenza **uno-a-uno**.
- Quando uno di essi viene completato, i valori rimanenti vengono scartati.

## 🔄 Differenza dalla Creation Function `zip`.

### Differenze di base.

|  | zip (Creation Function). | `zipWith` (Pipeable Operator). |
|---|---|---|
| **Dove si usa** | Usata come funzione indipendente | Usata nella catena `.pipe()`. |
| **Come si scrive**. | zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Primo flusso**. | Tratta tutti come uguali | Tratta come flusso principale |
| **Vantaggi**. | Semplice e di facile lettura | Facile da combinare con altri operatori |

### Esempi specifici di utilizzo

**Per un semplice accoppiamento, si raccomanda la Creation Function**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('Il nome è？', 'L'età è？', 'L'indirizzo è？');
const answers$ = of('Taro', '30', 'Tokyo');
const scores$ = of(10, 20, 30);

// Semplice e leggibile
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Punteggio: ${s}Punteggio`);
});
// L'uscita:
// Q: Il nome è？, A: Taro, Punteggio: 10Punteggio
// Q: L'età è？, A: 30, Punteggio: 20Punteggio
// Q: L'indirizzo è？, A: Tokyo, Punteggio: 30Punteggio
```

**Se si desidera aggiungere un processo di trasformazione al flusso principale, si consiglia di utilizzare la funzione Pipeable Operator**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Elenco dei compiti
const tasks$ = from([
  { id: 1, name: 'Segnalazione', priority: 'high' },
  { id: 2, name: 'Risposta alle e-mail', priority: 'low' },
  { id: 3, name: 'Preparazione di riunioni', priority: 'high' },
  { id: 4, name: 'Organizzare i documenti', priority: 'medium' }
]);

// Elenco dei responsabili (1Assegnato ogni secondo)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorEdizioni - Completare in una pipeline
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Solo ad alta priorità
    map(task => task.name),                     // Estrarre i nomi dei compiti
    zipWith(assignees$),                        // Assegnare la persona responsabile
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Assegnato a: ${assignment.assignee}`);
  });
// L'uscita:
// [Tempo] Segnalazione → Assegnato a: Alice
// [Tempo] Preparazione di riunioni → Assegnato a: Bob

// ❌ Creation FunctionEdizioni - Ridondante
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Assegnato a: ${assignment.assignee}`);
});
```

**Sincronizzazione di dati critici per l'ordine**.

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UICreare
const output = document.createElement('div');
output.innerHTML = '<h3>Gioco a quiz</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Elenco delle domande (preparato immediatamente)
const questions$ = from([
  'La capitale del Giappone è？',
  '1+1è？',
  'Quale pianeta è la Terra?？'
]);

// Elenco di risposte (simulando l'input dell'utente)：2(ogni secondo)
const answers$ = from(['Tokyo', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Elenco delle risposte corrette
const correctAnswers$ = from(['Tokyo', '2', '3']);

// ✅ Pipeable OperatorEdizioni - Elaborazione delle domande come mainstream
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>Domanda${result.no}:</strong> ${result.question}<br>
      <strong>Risposte:</strong> ${result.answer}<br>
      <strong>Risultato:</strong> ${result.isCorrect ? '✅ Risposta corretta！' : '❌ Risposta errata'}
    `;
    questionArea.appendChild(div);
  });
```

### Sintesi.

- **`zip`**: meglio se si vuole semplicemente mappare più flussi in ordine
- **`zipWith`**: ideale se si vuole unire il flusso principale con altri flussi in ordine garantito mentre si trasforma o si elabora il flusso principale

## ⚠️ Note.

### Per lunghezze diverse.

Quando l'Observable più breve termina, i valori rimanenti di quello più lungo vengono scartati.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// L'uscita: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'e'E'vengono scartati
```

### Accumulo di memoria.

Se un Observable continua a emettere valori, questi si accumulano in memoria finché l'altro non lo raggiunge.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Flussi veloci (100msper flusso)
const fast$ = interval(100).pipe(take(10));

// Flusso a bassa velocità (1(ogni secondo)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// L'uscita: [0, 0] (1secondi dopo), [1, 1] (2secondi dopo), [2, 2] (3secondi dopo)
// fast$i valori vengono memorizzati e aspettati
```

### Differenza da combineLatestWith.

ZipWith accoppia i valori nell'ordine corrispondente, mentre combineLatestWith combina i valori più recenti.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Accoppiati nell'ordine corrispondente
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// L'uscita: [0, 0], [1, 1]

// combineLatestWith: Combinare i valori più recenti
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// L'uscita: [0, 0], [1, 0], [2, 0], [2, 1]
```

## 📚 Operatori correlati.

- **[zip](/it/guide/creation-functions/combination/zip)** - Versione della Creation Function.
- **[combineLatestWith](/it/guide/operators/combination/combineLatestWith)** - Combina l'ultimo valore.
- **[withLatestFrom](/it/guide/operators/combination/withLatestFrom)** - Solo trigger mainstream
