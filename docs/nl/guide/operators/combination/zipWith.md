---
description: "zipWith is een RxJS join operator die de originele Observable uitlijnt en paart met de corresponderende geordende waarden van andere Observables. Het is ideaal voor order-kritische stream joins, zoals het combineren van de resultaten van parallelle verwerking met ordergaranties, het koppelen van ID's aan gegevens, het synchroniseren van gerelateerde gegevens die op verschillende tijdstippen zijn gepubliceerd, etc. De pipe operator versie is handig voor gebruik binnen pijplijnen."
---

# zipWith - paar overeenkomstige waarden

De ` zipWith` operator groepeert de **overeenkomstige geordende waarden** van de oorspronkelijke Observable en de andere gespecificeerde Observables.
Het wacht tot de waarden een voor een aankomen van alle Observable en maakt paren als ze klaar zijn.
Dit is de Pipeable Operator versie van `zip` van Creation Function.

## 🔰 Basissyntaxis en gebruik

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

// Uitgang:
// A - 0
// B - 10
// C - 20
// (Dis geen uitvoer omdat er geen overeenkomstige waarde is)
```

- Een paar wordt uitgevoerd door elke Observable **wanneer een van de waarden is voltooid**.
- Als een Observable is voltooid, worden de resterende waarden weggegooid.

[🌐 Officiële RxJS documentatie - zip](https://rxjs.dev/api/operators/zipWith)

## 💡 Typisch gebruikspatroon.

- **Het combineren van parallelle verwerkingsresultaten in gegarandeerde volgorde**: het koppelen van de resultaten van meerdere API-aanroepen.
- ID's aan gegevens koppelen**: gebruikers-ID's combineren met overeenkomstige profielgegevens.
- **Synchronisatie van streams**: synchronisatie van gerelateerde gegevens die op verschillende tijdstippen zijn uitgegeven.

## Praktische codevoorbeelden (met UI)

Voorbeeld van het op volgorde koppelen van een lijst met gebruikers-ID's en bijbehorende gebruikersnamen.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Uitvoergedeelte maken
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Praktische voorbeelden van:</h3>';
document.body.appendChild(output);

// GebruikerIDStream (onmiddellijk gepubliceerd)
const userIds$ = from([101, 102, 103, 104]);

// Gebruikersnaamstroom (uitgegeven elke1(elke seconde uitgegeven)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipen weergegeven
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 GebruikerID ${id}: ${name}`;
    output.appendChild(item);
  });

// Uitgang:
// 👤 GebruikerID 101: Alice
// 👤 GebruikerID 102: Bob
// 👤 GebruikerID 103: Carol
// (104wordt niet weergegeven omdat er geen overeenkomstige naam is)
```

- ID's en namen worden gekoppeld in een **één-op-één correspondentie**.
- Wanneer er één is voltooid, worden de resterende waarden verwijderd.

## Verschil met Creation Function `zip`.

### Basisverschillen.

|  | `zip` (Creation Function). | `zipWith` (Pipeable Operator) |
|---|---|---|
| **Waar gebruikt** | Gebruikt als zelfstandige functie | Gebruikt in `.pipe()` keten. |
| **Hoe te schrijven**. | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Eerste stroom**. | Alles als gelijk behandelen | Behandelen als hoofdstroom |
| **Voordelen**. | Eenvoudig en makkelijk te lezen | Makkelijk te combineren met andere operatoren |

### Specifieke voorbeelden van gebruik

**Voor eenvoudige koppelingen wordt de Creation Function aanbevolen**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('De naam is？', 'De leeftijd is？', 'Het adres is？');
const answers$ = of('Taro', '30', 'Tokio');
const scores$ = of(10, 20, 30);

// Eenvoudig en leesbaar
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Score: ${s}Score`);
});
// Uitgang:
// Q: De naam is？, A: Taro, Score: 10Score
// Q: De leeftijd is？, A: 30, Score: 20Score
// Q: Het adres is？, A: Tokio, Score: 30Score
```

**Als je een transformatieproces wilt toevoegen aan de hoofdstroom, wordt de Pipeable Operator aanbevolen**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Takenlijst
const tasks$ = from([
  { id: 1, name: 'Rapporteren', priority: 'high' },
  { id: 2, name: 'E-mails beantwoorden', priority: 'low' },
  { id: 3, name: 'Vergadering voorbereiden', priority: 'high' },
  { id: 4, name: 'Documenten organiseren', priority: 'medium' }
]);

// Lijst van verantwoordelijke personen (1Elke seconde toegewezen)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorEdities - Voltooien in één pijplijn
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Alleen hoge prioriteit
    map(task => task.name),                     // Namen van taken uitpakken
    zipWith(assignees$),                        // Verantwoordelijke toewijzen
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Toegewezen aan: ${assignment.assignee}`);
  });
// Uitgang:
// [Tijd] Rapporteren → Toegewezen aan: Alice
// [Tijd] Vergadering voorbereiden → Toegewezen aan: Bob

// ❌ Creation FunctionEdities - Overbodig
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
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Toegewezen aan: ${assignment.assignee}`);
});
```

**Synchronisatie van orderkritische gegevens**.

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UIMaak
const output = document.createElement('div');
output.innerHTML = '<h3>Quiz</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Lijst met vragen (onmiddellijk voorbereid)
const questions$ = from([
  'De hoofdstad van Japan is？',
  '1+1is？',
  'Welke planeet is de aarde?？'
]);

// Antwoordenlijst (simuleren invoer gebruiker)：2(elke seconde)
const answers$ = from(['Tokio', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Lijst met juiste antwoorden
const correctAnswers$ = from(['Tokio', '2', '3']);

// ✅ Pipeable OperatorEdities - Verwerk vragen als mainstream
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
      <strong>Vraag${result.no}:</strong> ${result.question}<br>
      <strong>Antwoorden:</strong> ${result.answer}<br>
      <strong>Resultaat:</strong> ${result.isCorrect ? '✅ Juist antwoord！' : '❌ Onjuist antwoord'}
    `;
    questionArea.appendChild(div);
  });
```

### Samenvatting.

- **`zip`**: het beste als u alleen meerdere streams in volgorde wilt mappen
- **`zipWith`**: ideaal als je de hoofdstroom wilt samenvoegen met andere stromen in gegarandeerde volgorde tijdens het transformeren of verwerken van de hoofdstroom

## ⚠️ Opmerkingen.

### Voor verschillende lengtes.

Als de kortere Observable is voltooid, worden de resterende waarden van de langere weggegooid.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Uitgang: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'en'E'worden verwijderd
```

### Geheugenaccumulatie.

Als een Observable waarden blijft afgeven, hopen de waarden zich op in het geheugen totdat de andere het inhaalt.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Snelle streams (100msper stream)
const fast$ = interval(100).pipe(take(10));

// Stream met lage snelheid (1(elke seconde)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Uitgang: [0, 0] (1seconden later), [1, 1] (2seconden later), [2, 2] (3seconden later)
// fast$waarden worden opgeslagen in het geheugen en er wordt gewacht op
```

### Verschil met combineLatestWith.

`zipWith` koppelt de waarden in de overeenkomstige volgorde, terwijl `combineLatestWith` de nieuwste waarden combineert.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Gekoppeld in overeenkomstige volgorde
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Uitgang: [0, 0], [1, 1]

// combineLatestWith: De laatste waarden combineren
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Uitgang: [0, 0], [1, 0], [2, 0], [2, 1]
```

## Gerelateerde operatoren.

- **[zip](/nl/guide/creation-functions/combination/zip)** - Versie Creation Function.
- **[combineLatestWith](/nl/guide/operators/combination/combineLatestWith)** - Laatste waarde combineren.
- **[withLatestFrom](/nl/guide/operators/combination/withLatestFrom)** - alleen mainstream triggers
