---
description: "De zip Creation Function maakt paren van waarden in corresponderende volgorde van meerdere Observables en output op het moment dat alle bronnen één voor één een waarde hebben uitgezonden. Geschikt voor datasynchronisatie en combinatie van parallelle verwerkingsresultaten. Uitleg over typeveilige implementatie in TypeScript met praktische codevoorbeelden."
---

# zip - Corresponderende waarden koppelen

`zip` is een Creation Function die **waarden in corresponderende volgorde** van meerdere Observables samenbrengt en als array of tuple output.
Het wacht tot er van alle bron-Observables één voor één waarden zijn aangekomen en maakt paren wanneer ze compleet zijn.


## Basissyntaxis en gebruik

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

- Op het moment dat elke Observable één voor één een waarde emit, wordt een paar gemaakt en uitgevoerd.
- Zelfs als één kant vertraagd is, wacht het tot beide compleet zijn.

[🌐 RxJS Officiële Documentatie - `zip`](https://rxjs.dev/api/index/function/zip)


## Typische toepassingspatronen

- **Request en response koppelen**
- **ID's en corresponderende data synchroon koppelen**
- **Meerdere parallel verwerkte streams bundelen tot één set**


## Praktisch codevoorbeeld (met UI)

Een voorbeeld van het **combineren en weergeven** van verschillende databronnen (fruit en prijzen).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Outputgebied aanmaken
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisch voorbeeld van zip:</h3>';
document.body.appendChild(output);

// Fruitnaam stream
const fruits$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// Prijsstream (emit om de 2 seconden)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// zip gebruiken voor weergave
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - €${price}`;
  output.appendChild(item);
});
```

- Fruit- en prijslijsten worden weergegeven als paren **op het moment dat ze in 1-op-1 correspondentie compleet zijn**.
- Als één kant onvoldoende is, wordt het op dat moment niet uitgevoerd.


## Gerelateerde operators

- **[zipWith](/nl/guide/operators/combination/zipWith)** - Pipeable Operator versie (voor gebruik in pipeline)
- **[combineLatest](/nl/guide/creation-functions/combination/combineLatest)** - Creation Function die laatste waarden combineert
- **[withLatestFrom](/nl/guide/operators/combination/withLatestFrom)** - Alleen hoofdstream triggert
