---
description: "De filter operator sorteert waarden in een stroom gebaseerd op een gespecificeerde voorwaardelijke functie, en laat alleen waarden door die voldoen aan de voorwaarde. Het kan gebruikt worden als een Type Guard functie (type predicaat) in TypeScript, en legt ook het verschil uit tussen het en buffer, en de voorbehouden om van een predicaat functie een pure functie te maken. Deze paragraaf legt ook het verschil uit tussen buffers en pure functies."
---

# filter - laat alleen waarden door die aan de voorwaarden voldoen

De `filter` operator sorteert waarden in een stroom op basis van een gespecificeerde voorwaardelijke functie en laat alleen waarden door die aan de voorwaarde voldoen.

## 🔰 Basissyntaxis en gebruik

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Uitgangen: 2, 4, 6, 8, 10
```

- Alleen waarden die voldoen aan de voorwaarde worden doorgelaten.
- Werkt hetzelfde als `Array.prototype.filter()` op arrays, maar is sequentieel op de Observable.

[🌐 Officiële RxJS documentatie - `filter`](https://rxjs.dev/api/operators/filter)

## Typisch gebruikspatroon.

- Validatie van formulierinvoerwaarden
- Alleen gegevens van een specifiek type of specifieke structuur toestaan
- Filteren van sensorgebeurtenissen en stroomgegevens

## Praktische codevoorbeelden (met UI)

Alleen een lijst in realtime weergeven als het ingevoerde getal even is.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter Praktische voorbeelden van:';
document.body.appendChild(title);

// Invoervelden maken
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Numerieke waarden invoeren';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Uitvoergebied maken
const output = document.createElement('div');
document.body.appendChild(output);

// Gebeurtenisstroom invoeren
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Detectie even getallen: ${evenNumber}`;
    output.prepend(item);
  });

```

- Alleen weergegeven in de uitvoer als het getal even is.
- Oneven of ongeldige invoer wordt genegeerd.

> [!WARNING] 本番コードでの注意

> In het bovenstaande voorbeeld is het afmelden van `fromEvent` weggelaten voor de eenvoud van de uitleg. Gebruik in echte code `takeUntil(destroy$)`, `take(N)` of `Subscription.unsubscribe()` om de levenscyclus expliciet te beheren. Meer informatie: [Moeilijkheden overwinnen: levenscyclusbeheer](/nl/guide/overcoming-difficulties/lifecycle-management.md)

## Verschillen met buffer

| Bediening | Werking | Uitvoeren. |
|---|---|---|
| `filter`. | Verwijder waarden die niet **overeenkomen** met de voorwaarde. | Individuele waarden `T`. |
| buffer`. | Sla** waarden op in een array**. | Matrix `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Alleen waarden die voldoen aan de voorwaarden worden doorgelaten
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Uitgangen: filter: 0
  // Uitgangen: filter: 2
  // Uitgangen: filter: 4
});

// buffer - Slaat waarden op als een matrix
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Uitgangen: buffer: [0, 1]
  // Uitgangen: buffer: [2, 3, 4]
});
```

## ⚠️ Opmerkingen.

### 1. Predicaatfuncties moeten zuivere functies zijn.

Predicaatfuncties met neveneffecten kunnen onverwacht gedrag veroorzaken wanneer de stream opnieuw wordt onderschreven.

```ts
// ❌ Slecht voorbeeld: Neveneffecten Ja
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Neveneffect
    return x > 10;
  })
).subscribe();

// ✅ Goed voorbeeld: Zuivere functie
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. Gebruik als functie voor typebewaking

Je kunt de functie schrijven om een TypeScript type predicaat terug te geven (`x is T`) om het type te beperken na het passeren van `filter`.

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Gebruikt als typebewakingsfunctie
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email Is geen typebewakingsfunctie string Wordt afgeleid als een type
});
```

> [!TIP] 型ガードの効果

> Door het typepredicaat `user is User & { email: string }` terug te geven, maakt `user` na `filter` `email` een verplichte eigenschap. Oproepen als `user.email.toLowerCase()` kunnen zonder typefouten geschreven worden.

## Gerelateerde operatoren.

- take](/nl/guide/operators/filtering/take) - alleen de eerste N waarden worden meegenomen.
- first](/nl/guide/operators/filtering/first) - haal alleen de eerste waarde (kan ook voorwaardelijk zijn).
- distinct](/nl/guide/operators/filtering/distinct) - dubbele waarden uitsluiten
- distinctUntilChanged](/nl/guide/operators/filtering/distinctUntilChanged) - sluit hetzelfde als de laatste waarde uit

## Samenvatting.

De `filter` operator is de meest basale filtertool in RxJS.

- Alleen waarden die aan de voorwaarden voldoen worden doorgelaten.
- ✅ Kan op dezelfde manier gebruikt worden als `.filter()` voor arrays.
- ✅ Kan ook gebruikt worden als TypeScript type guard.
- ⚠️ Predicaatfuncties moeten pure functies zijn
- ⚠️ Gelijkaardige naam maar ander gebruik dan `buffer` (individuele waarden vs. arrays)
