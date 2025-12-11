---
description: zipWith is een RxJS combinatie-operator die de originele Observable paart met andere Observables in corresponderende volgorde. Het is de Pipeable Operator versie van Creation Function zip, en is ideaal voor situaties waar volgorde belangrijk is, zoals quizspellen (vragen en antwoorden), taaktoewijzingen (gebruikers en taken), stoeltoewijzingen (passagiers en stoelnummers), etc.
titleTemplate: ':title | RxJS'
---

# zipWith - Paren op basis van volgorde (Binnen Pipeline)

De `zipWith` operator paart de originele Observable met de gespecificeerde andere Observables **in corresponderende volgorde** om een nieuwe stream te vormen.
Dit is de Pipeable Operator versie van de Creation Function `zip`.

## 🔰 Basissyntax en gebruik

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C', 'D');
const source2$ = interval(500).pipe(take(4)); // 0, 1, 2, 3

source1$
  .pipe(
    zipWith(source2$),
    map(([letter, num]) => `${letter}${num}`)
  )
  .subscribe(console.log);

// Output:
// A0 (na 500ms)
// B1 (na 1000ms)
// C2 (na 1500ms)
// D3 (na 2000ms)
```

- Paart **waarden in corresponderende volgorde** van elke Observable, één voor één.
- **Wacht totdat alle Observables waarden hebben geëmitteerd in corresponderende volgorde** voordat het paar wordt uitgegeven.
- Wanneer enige Observable voltooit, voltooit de gehele stream.

[🌐 RxJS Officiële Documentatie - `zipWith`](https://rxjs.dev/api/operators/zipWith)


## 💡 Typische gebruikspatronen

- **Quizspel vraag en antwoord paren**: Sequentiële vragen paren met gebruikersantwoorden
- **Taaktoewijzing**: Gebruikerslijsten paren met takenlijsten in volgorde
- **Stoeltoewijzing**: Passagiers paren met stoelnummers in volgorde
- **Parallelle verwerkingsresultaten consolideren**: Resultaten van meerdere API-aanroepen in volgorde combineren


## 🧠 Praktisch codevoorbeeld (met UI)

Voorbeeld van een quizspel waar vragen en gebruikersantwoorden in volgorde worden gepaard en gescoord.

```ts
import { fromEvent, of, from } from 'rxjs';
import { zipWith, map, take, scan } from 'rxjs';

// Bouw de UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>zipWith Praktisch Voorbeeld: Quizspel</h3>
  <div id="question" style="font-size: 18px; margin: 10px 0;">Vragen laden...</div>
  <div>
    <button id="answer-a">A</button>
    <button id="answer-b">B</button>
    <button id="answer-c">C</button>
  </div>
  <div id="result" style="margin-top: 10px;"></div>
  <div id="score" style="margin-top: 10px; font-weight: bold;"></div>
`;
document.body.appendChild(container);

const questionDiv = document.getElementById('question')!;
const resultDiv = document.getElementById('result')!;
const scoreDiv = document.getElementById('score')!;

const buttonA = document.getElementById('answer-a') as HTMLButtonElement;
const buttonB = document.getElementById('answer-b') as HTMLButtonElement;
const buttonC = document.getElementById('answer-c') as HTMLButtonElement;

// Vragenlijst (met correcte antwoorden)
interface Question {
  id: number;
  text: string;
  correct: string;
}

const questions: Question[] = [
  { id: 1, text: 'Waar staat "Rx" in RxJS voor?', correct: 'A' },
  { id: 2, text: 'Wat vertegenwoordigt Observable?', correct: 'B' },
  { id: 3, text: 'Wat doet subscribe?', correct: 'C' }
];

// Vragenstream
const questions$ = of(...questions);

let currentQuestionIndex = 0;
questions$.subscribe(q => {
  if (currentQuestionIndex === 0) {
    questionDiv.textContent = `V${q.id}: ${q.text}`;
  }
});

// Gebruikersantwoord stream (knopklikken)
const getAnswer = () => new Promise<string>((resolve) => {
  const handleClick = (answer: string) => {
    resolve(answer);
    buttonA.removeEventListener('click', handleA);
    buttonB.removeEventListener('click', handleB);
    buttonC.removeEventListener('click', handleC);
  };
  const handleA = () => handleClick('A');
  const handleB = () => handleClick('B');
  const handleC = () => handleClick('C');
  buttonA.addEventListener('click', handleA);
  buttonB.addEventListener('click', handleB);
  buttonC.addEventListener('click', handleC);
});

const answers$ = from(
  Promise.all(questions.map(() => getAnswer()))
);

// Paar en beoordeel vragen met antwoorden
questions$
  .pipe(
    zipWith(answers$),
    map(([question, answer]) => ({
      question: question.text,
      answer,
      correct: question.correct,
      isCorrect: answer === question.correct
    })),
    scan((acc, result) => ({
      ...result,
      totalScore: acc.totalScore + (result.isCorrect ? 1 : 0)
    }), { totalScore: 0 } as any)
  )
  .subscribe((result) => {
    const status = result.isCorrect ? '✅ Correct' : '❌ Fout';
    resultDiv.innerHTML += `<div>${status}: ${result.question} - Jouw antwoord: ${result.answer}</div>`;
    scoreDiv.textContent = `Huidige score: ${result.totalScore} / ${currentQuestionIndex + 1}`;
    currentQuestionIndex++;

    // Toon volgende vraag
    if (currentQuestionIndex < questions.length) {
      questionDiv.textContent = `V${questions[currentQuestionIndex].id}: ${questions[currentQuestionIndex].text}`;
    } else {
      questionDiv.textContent = 'Alle vragen voltooid!';
      buttonA.disabled = true;
      buttonB.disabled = true;
      buttonC.disabled = true;
    }
  });
```

- Elke keer dat een gebruiker antwoordt, wordt het **gepaard met de corresponderende vraag** en gescoord.
- Volgorde is gegarandeerd, dus de correspondentie blijft behouden: **Antwoord 1 voor Vraag 1, Antwoord 2 voor Vraag 2**, enzovoort.


## 🔄 Verschil met Creation Function `zip`

### Basisverschillen

| | `zip` (Creation Function) | `zipWith` (Pipeable Operator) |
|:---|:---|:---|
| **Gebruikslocatie** | Gebruikt als onafhankelijke functie | Gebruikt binnen `.pipe()` keten |
| **Syntax** | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Eerste stream** | Behandelt allemaal gelijk | Behandelt als hoofdstream |
| **Voordeel** | Eenvoudig en leesbaar | Eenvoudig te combineren met andere operators |

### Specifieke gebruiksvoorbeelden

**Creation Function wordt aanbevolen voor alleen eenvoudige paren**

```ts
import { zip, of } from 'rxjs';

const names$ = of('Alice', 'Bob', 'Charlie');
const ages$ = of(25, 30, 35);
const cities$ = of('Tokyo', 'Osaka', 'Kyoto');

// Eenvoudig en leesbaar
zip(names$, ages$, cities$).subscribe(([name, age, city]) => {
  console.log(`${name} (${age} jaar oud) - ${city}`);
});
// Output:
// Alice (25 jaar oud) - Tokyo
// Bob (30 jaar oud) - Osaka
// Charlie (35 jaar oud) - Kyoto
```

**Pipeable Operator wordt aanbevolen bij het toevoegen van transformatieverwerking aan hoofdstream**

```ts
import { of } from 'rxjs';
import { zipWith, map, filter } from 'rxjs';

const users$ = of(
  { id: 1, name: 'Alice', active: true },
  { id: 2, name: 'Bob', active: false },
  { id: 3, name: 'Charlie', active: true }
);

const tasks$ = of('Taak A', 'Taak B', 'Taak C');

// ✅ Pipeable Operator versie - voltooid in één pipeline
users$
  .pipe(
    filter(user => user.active),    // Alleen actieve gebruikers
    map(user => user.name),         // Haal alleen naam op
    zipWith(tasks$)                 // Paar met taken
  )
  .subscribe(([user, task]) => {
    console.log(`Wijs ${task} toe aan ${user}`);
  });
// Output:
// Wijs Taak A toe aan Alice
// Wijs Taak B toe aan Charlie

// ❌ Creation Function versie - wordt omslachtig
import { zip } from 'rxjs';
zip(
  users$.pipe(
    filter(user => user.active),
    map(user => user.name)
  ),
  tasks$
).subscribe(([user, task]) => {
  console.log(`Wijs ${task} toe aan ${user}`);
});
```

### Samenvatting

- **`zip`**: Optimaal voor eenvoudig paren van meerdere streams
- **`zipWith`**: Optimaal wanneer u de hoofdstream wilt transformeren/verwerken terwijl u deze paart met andere streams


## ⚠️ Belangrijke opmerkingen

### Voltooiingstiming

Wanneer enige Observable voltooit, voltooit de gehele stream.

```ts
import { of, interval } from 'rxjs';
import { zipWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    interval(1000).pipe(take(2)),  // Emitteert slechts 2 waarden
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Voltooid')
});
// Output: [1, 0] → [2, 1] → ✅ Voltooid
// * interval$ emitteerde slechts 2 waarden en voltooide, dus 3 wordt niet gepaard
```

### Synchronisatie van emissietiming

`zipWith` wacht **totdat alle Observables waarden hebben geëmitteerd in corresponderende volgorde**.

```ts
import { interval } from 'rxjs';
import { zipWith, take, map } from 'rxjs';

const fast$ = interval(100).pipe(take(5), map(i => `Snel: ${i}`));
const slow$ = interval(1000).pipe(take(5), map(i => `Langzaam: ${i}`));

fast$
  .pipe(zipWith(slow$))
  .subscribe(console.log);
// Output (elke 1 seconde):
// ['Snel: 0', 'Langzaam: 0']
// ['Snel: 1', 'Langzaam: 1']
// ['Snel: 2', 'Langzaam: 2']
// ['Snel: 3', 'Langzaam: 3']
// ['Snel: 4', 'Langzaam: 4']
// * fast$ is snel, maar wacht op slow$ om te emitteren, dus paren worden elke seconde uitgegeven
```

### Verschil met combineLatestWith

`combineLatestWith` **combineert altijd** de laatste waarden, terwijl `zipWith` **paart op basis van volgorde**.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(100).pipe(take(3)); // 0, 1, 2
const source2$ = interval(200).pipe(take(2)); // 0, 1

// zipWith: Paar op volgorde
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Output: [0, 0] → [1, 1] → Voltooid
// * Aangezien source2$ voltooide, wordt source1$'s 2 niet gepaard

// combineLatestWith: Combineer laatste waarden
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Output: [0, 0] → [1, 0] → [1, 1] → [2, 1]
// * Geeft laatste waardecombinatie uit telkens wanneer één emitteert
```

### Foutafhandeling

Als er een fout optreedt in enige Observable, eindigt de gehele stream met een fout.

```ts
import { throwError, of } from 'rxjs';
import { zipWith, catchError } from 'rxjs';

of(1, 2, 3).pipe(
  zipWith(
    throwError(() => new Error('Fout opgetreden')).pipe(
      catchError(err => of('Fout hersteld'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Output: [1, 'Fout hersteld']
```


## 📚 Gerelateerde operators

- **[zip](/nl/guide/creation-functions/combination/zip)** - Creation Function versie
- **[combineLatestWith](/nl/guide/operators/combination/combineLatestWith)** - Combineer altijd laatste waarden
- **[withLatestFrom](/nl/guide/operators/combination/withLatestFrom)** - Combineer alleen wanneer hoofdstream emitteert
