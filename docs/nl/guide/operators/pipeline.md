---
description: "Leer RxJS pipeline-constructie met de pipe-methode: Transformeer, filter en combineer datastromen declaratief met uitgebreide operator chaining voorbeelden"
---

# Wat is een RxJS Pipeline?

Pipelining in RxJS is een mechanisme om een reeks bewerkingen (operators) in volgorde toe te passen op een Observable. Pipelining stelt u in staat om datastromen in meerdere fasen te transformeren, filteren en combineren, waardoor u de datastroom kunt controleren in een declaratieve programmeerstijl.

## Basisstructuur van een Pipeline

[📘 RxJS Officieel: pipe()](https://rxjs.dev/api/index/function/pipe)

De RxJS `pipe()` methode wordt gebruikt om een pipeline te bouwen. De syntax is als volgt.

```ts
import { Observable } from 'rxjs';
import { map, filter, tap } from 'rxjs';

const source$: Observable<number> = // Een Observable
source$.pipe(
  // Keten meerdere operators
  operator1(),
  operator2(),
  operator3(),
  // ...
).subscribe(value => {
  // Verwerk het resultaat
});
```

## Praktische voorbeelden

### Basis dataconversie

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// Stream van getallen
const numbers$ = of(1, 2, 3, 4, 5);

// Bouw een pipeline
numbers$.pipe(
  // Laat alleen even getallen door
  filter(n => n % 2 === 0),
  // Verdubbel de waarde
  map(n => n * 2)
).subscribe(
  value => console.log(`Resultaat: ${value}`)
);

// Uitvoer:
// Resultaat: 4
// Resultaat: 8
```

### Complexe dataverwerking

```ts
import { fromEvent, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
};
type Post = {
  userId: number;
  id: number;
  title: string;
  body: string;
};

// Maak DOM-elementen
const searchButton = document.createElement('button');
searchButton.innerText = 'Zoeken';
document.body.appendChild(searchButton);

const resultBox = document.createElement('div');
resultBox.id = 'results';
document.body.appendChild(resultBox);

// API-verzoek bij knopklik
fromEvent(searchButton, 'click')
  .pipe(
    switchMap(() =>
      // Eerste API-aanroep
      ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
        // Tweede API-aanroep om berichten van gebruiker op te halen
        switchMap((user) => {
          const header = document.createElement('h3');
          header.textContent = `Gebruiker: ${user.name}`;
          resultBox.innerHTML = ''; // Wis vorige resultaten
          resultBox.appendChild(header);

          return ajax.getJSON<Post[]>(
            `https://jsonplaceholder.typicode.com/posts?userId=${user.id}`
          );
        }),
        // Haal alleen de eerste 3 berichten op
        map((posts) => posts.slice(0, 3))
      )
    )
  )
  .subscribe((posts) => {
    // Toon berichten op het scherm
    resultBox.innerHTML += '<h4>Berichten:</h4>';
    posts.forEach((post) => {
      const div = document.createElement('div');
      div.innerHTML = `<strong>${post.title}</strong><p>${post.body}</p>`;
      resultBox.appendChild(div);
    });
  });

```


## Voordelen van de Pipeline

Laten we eerst kijken naar code geschreven op een imperatieve manier. Zoals hierna getoond, stelt RxJS pipelining u in staat om het te herschrijven in een leesbaarder en beter te onderhouden vorm terwijl de intentie van het proces duidelijk wordt gemaakt.

### 1. Verbeterde leesbaarheid en onderhoudbaarheid

```ts
// Verwerking in imperatieve stijl
const data = [
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
];

const activeItems = [];
for (const item of data) {
  if (item.active) {
    activeItems.push({ ...item, label: `Item #${item.id}` });
  }
}
activeItems.sort((a, b) => a.id - b.id);

const div1 = document.createElement('div');
div1.innerHTML = '<h3>Imperatieve stijl</h3>';
activeItems.forEach(item => {
  const p = document.createElement('p');
  p.textContent = item.label;
  div1.appendChild(p);
});
document.body.appendChild(div1);
```
⬇️⬇️⬇️
```ts
import { of } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>Verbeterde leesbaarheid en onderhoudbaarheid</h3>';
document.body.appendChild(output);

of(
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
).pipe(
  filter(item => item.active),
  map(item => ({ ...item, label: `Item #${item.id}` })),
  toArray(),
  map(array => array.sort((a, b) => a.id - b.id))
).subscribe(sorted => {
  sorted.forEach(item => {
    const div = document.createElement('div');
    div.textContent = item.label;
    output.appendChild(div);
  });
});
```

Pipelining maakt de datastroom duidelijk en elimineert de noodzaak om variabelen opnieuw toe te wijzen of tussenliggende toestanden te beheren.



Procedurele code zoals hierboven kan beknopt worden geschreven in een declaratieve stijl door RxJS pipelining te gebruiken. Een voorbeeld wordt hieronder getoond.

### 2. Declaratieve programmeerstijl

Pipelining bevordert een declaratieve stijl die expliciet aangeeft "wat te doen". Dit maakt de intentie van de code duidelijker.

```ts
// Verwerking in procedurele stijl
const usersList = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

const activeUsers2 = [];
for (const user of usersList) {
  if (user.status === 'active') {
    const name = `${user.firstName} ${user.lastName}`;
    activeUsers2.push({ name, email: user.email });
  }
}

const div2 = document.createElement('div');
div2.innerHTML = '<h3>Procedurele stijl</h3>';
activeUsers2.forEach(user => {
  const p = document.createElement('p');
  p.textContent = `${user.name} (${user.email})`;
  div2.appendChild(p);
});
document.body.appendChild(div2);
```

⬇️⬇️⬇️

```ts
// Declaratieve programmeerstijl
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

const out2 = document.createElement('div');
out2.innerHTML = '<h3>Declaratieve stijl</h3>';
document.body.appendChild(out2);

const users = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

from(users).pipe(
  filter(user => user.status === 'active'),
  map(user => ({
    name: `${user.firstName} ${user.lastName}`,
    email: user.email
  }))
).subscribe(user => {
  const div = document.createElement('div');
  div.textContent = `${user.name} (${user.email})`;
  out2.appendChild(div);
});
```


Laten we hier ook code nemen die verwerking op een procedurele manier beschrijft en deze reorganiseren met pipelining. Complexe verwerking kan eenvoudig worden opgebouwd door individuele operators te combineren.

### 3. Composeerbaarheid

Pipelining stelt u in staat complexe verwerking te bouwen door kleine bewerkingen te combineren.

```ts
// Procedurele (imperatieve) stijl verwerking
const rawUsers = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const activeUsers = [];
for (const user of rawUsers) {
  if (user.status === 'active') {
    const fullName = `${user.firstName} ${user.lastName}`;
    activeUsers.push({ ...user, fullName });
  }
}
activeUsers.sort((a, b) => a.fullName.localeCompare(b.fullName));

const div0 = document.createElement('div');
div0.innerHTML = '<h3>Procedurele stijl</h3>';
activeUsers.forEach(user => {
  const p = document.createElement('p');
  p.textContent = user.fullName;
  div0.appendChild(p);
});
document.body.appendChild(div0);
```

⬇️⬇️⬇️

```ts
// Declaratieve programmeerstijl
import { from } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const out3 = document.createElement('div');
out3.innerHTML = '<h3>Composeerbaarheid</h3>';
document.body.appendChild(out3);

const users3 = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const filterActive = filter((user: any) => user.status === 'active');
const formatFullName = map((user: any) => ({ ...user, fullName: `${user.firstName} ${user.lastName}` }));
const collectAndSort = toArray();
const sortByName = map((users: any[]) => users.sort((a, b) => a.fullName.localeCompare(b.fullName)));

from(users3).pipe(
  filterActive,
  formatFullName,
  collectAndSort,
  sortByName
).subscribe(users => {
  users.forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.fullName;
    out3.appendChild(div);
  });
});
```

## Pipeline optimalisatietechnieken

### 1. Belang van operatorvolgorde

Operatorvolgorde heeft een significante impact op zowel prestatie als functionaliteit.

```ts
// Inefficiënt: map wordt toegepast op alle elementen
observable$.pipe(
  map(x => expensiveTransformation(x)),
  filter(x => x > 10)
)

// Efficiënt: filter wordt eerst uitgevoerd, waardoor elementen om te transformeren worden verminderd
observable$.pipe(
  filter(x => x > 10),
  map(x => expensiveTransformation(x))
)
```

### 2. Aangepaste pipelines maken

Complexe verwerking kan worden geëxtraheerd naar herbruikbare pipelines.

```ts
import { Observable, pipe } from 'rxjs';
import { filter, map } from 'rxjs';

// Aangepaste pipeline functie
export function filterAndTransform<T, R>(
  filterFn: (value: T) => boolean,
  transformFn: (value: T) => R
) {
  return pipe(
    filter(filterFn),
    map(transformFn)
  );
}

// Gebruiksvoorbeeld
observable$.pipe(
  filterAndTransform(
    x => x > 10,
    x => x * 2
  )
).subscribe(console.log);
```

## Veelgemaakte fouten met pipelines

### 1. Verkeerde operatorvolgorde

```ts
// ❌ Als je filter toepast vóór debounceTime,
// wordt filter uitgevoerd voor elke invoer, waardoor het effect van debounce vermindert
inputEvents$.pipe(
  filter(text => text.length > 2),
  debounceTime(300)
)

// ✅ Pas eerst debounceTime toe
inputEvents$.pipe(
  debounceTime(300),
  filter(text => text.length > 2)
)
```

### 2. Bijwerkingen in de pipeline

```ts
// ❌ Direct bijwerkingen uitvoeren in de pipeline
observable$.pipe(
  map(data => {
    // Bijwerkingen (slecht voorbeeld)
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
    return data;
  })
)

// ✅ Gebruik de tap operator
observable$.pipe(
  tap(data => {
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
  }),
  // Voer datatransformatie uit met map
  map(data => transformData(data))
)
```

## Samenvatting

RxJS pipelines zijn een krachtig mechanisme voor het beheren van complexe asynchrone datastromen op een declaratieve en composeerbare manier. Goed ontworpen pipelines kunnen de leesbaarheid, onderhoudbaarheid en herbruikbaarheid van code aanzienlijk verbeteren.

Bij het ontwerpen van pipelines is het een goed idee om de volgende punten in gedachten te houden:

1. Kies de meest efficiënte volgorde van operators
2. Extraheer en hergebruik veelvoorkomende pipeline-patronen
3. Isoleer bijwerkingen met `tap` operators
4. Zorg ervoor dat elke stap in de pipeline een enkele verantwoordelijkheid heeft

Een dergelijke pipeline-georiënteerde aanpak is vooral krachtig in scenario's zoals complexe UI-gebeurtenisverwerking, API-verzoeken en statusbeheer.
