---
description: "De forkJoin Creation Function output, nadat meerdere Observables allemaal zijn voltooid, de respectievelijke laatste waarden samengevoegd als array of object. Optimaal voor het in één keer ophalen van resultaten van parallelle API-aanroepen. Uitleg over type-inferentie in TypeScript met praktische codevoorbeelden."
---

# forkJoin - Alle laatste waarden samengevoegd output

`forkJoin` is een Creation Function die **nadat meerdere Observables allemaal zijn voltooid, de respectievelijke laatste waarden samengevoegd als array of object output**.
Zeer handig wanneer je "alle resultaten samen wilt gebruiken nadat ze compleet zijn".


## Basissyntaxis en gebruik

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('Gebruiker A').pipe(delay(1000));
const posts$ = of('Berichtenlijst').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Output:
// Gebruiker A Berichtenlijst
```

- Wacht tot alle Observables `complete` zijn.
- **Alleen de laatste emissiewaarde** van elke Observable wordt samengevoegd en uitgevoerd.

[🌐 RxJS Officiële Documentatie - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## Typische toepassingspatronen

- **Meerdere API-requests parallel uitvoeren en alle resultaten samenbrengen**
- **Meerdere datasets die nodig zijn bij initiële lading in één keer ophalen**
- **Gerelateerde data samen ophalen en gebundeld weergeven op scherm**


## Praktisch codevoorbeeld (met UI)

Simuleert meerdere API-requests en toont alle resultaten samen wanneer ze compleet zijn.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Outputgebied aanmaken
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisch voorbeeld van forkJoin:</h3>';
document.body.appendChild(output);

// Dummy data streams
const user$ = of({ id: 1, name: 'Jan Jansen' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Bericht 1' }, { id: 2, title: 'Bericht 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Zonnig' }).pipe(delay(1000));

// Laadmelding
const loading = document.createElement('div');
loading.textContent = 'Gegevens laden...';
loading.style.color = 'blue';
output.appendChild(loading);

// Na voltooiing van alle requests samen output
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `Gebruiker: ${result.user.name}, Weer: ${result.weather.condition}, Aantal berichten: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Eerst laadmelding weergeven,
- Wanneer alle data compleet is, resultaten samen renderen.
