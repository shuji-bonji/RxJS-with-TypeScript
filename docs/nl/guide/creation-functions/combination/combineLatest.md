---
description: "Uitleg over het combineren van de laatste waarden van meerdere Observables met de combineLatest Creation Function. Telkens wanneer een bron een waarde emit, wordt de nieuwste combinatie uitgevoerd, daarom optimaal voor synchronisatie van UI en formulierinvoer, monitoring van meerdere condities. Inclusief type-inferentie in TypeScript en praktische voorbeelden."
---

# combineLatest - Laatste waarden combineren

`combineLatest` is een Creation Function die **alle laatste waarden van meerdere Observables samenbrengt en output**.
Telkens wanneer een nieuwe waarde van een bron-Observable wordt uitgezonden, wordt een resultaat met alle laatste waarden uitgezonden.

## Basissyntaxis en gebruik

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Output:
// C 1
// C 2
// C 3
```

- Nadat elke Observable **minstens één waarde heeft uitgezonden**, wordt de gecombineerde waarde uitgevoerd.
- Telkens wanneer er een nieuwe waarde in een van beiden komt, wordt het laatste paar opnieuw uitgevoerd.

[🌐 RxJS Officiële Documentatie - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## Typische toepassingspatronen

- **Real-time validatie van formulierinvoer** (bijv. gelijktijdige monitoring van naam en e-mailadres)
- **Synchronisatie van de status van meerdere streams** (bijv. integratie van sensorwaarden of apparaatstatussen)
- **Data-fetching met afhankelijkheden** (bijv. combinatie van gebruikers-ID en instellingen-ID)

## Praktisch codevoorbeeld (met UI)

Combineert en toont altijd de nieuwste status van twee formulierinvoervelden.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Outputgebied aanmaken
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisch voorbeeld van combineLatest:</h3>';
document.body.appendChild(output);

// Formuliervelden aanmaken
const nameInput = document.createElement('input');
nameInput.placeholder = 'Voer naam in';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'Voer e-mail in';
document.body.appendChild(emailInput);

// Observable voor elke invoer
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Laatste invoerwaarden combineren
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Naam:</strong> ${name}</div>
    <div><strong>E-mail:</strong> ${email}</div>
  `;
});
```

- Wanneer je in een van de velden invoert, worden **de nieuwste twee invoerstatussen onmiddellijk weergegeven**.
- Door `startWith('')` te gebruiken, kun je vanaf het begin het gecombineerde resultaat verkrijgen.


## Gerelateerde operators

- **[combineLatestWith](/nl/guide/operators/combination/combineLatestWith)** - Pipeable Operator versie (voor gebruik in pipeline)
- **[withLatestFrom](/nl/guide/operators/combination/withLatestFrom)** - Alleen hoofdstream triggert
- **[zip](/nl/guide/creation-functions/combination/zip)** - Creation Function die corresponderende waarden koppelt
