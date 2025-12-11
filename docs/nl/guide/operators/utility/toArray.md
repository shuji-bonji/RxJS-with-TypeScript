---
description: toArray is een RxJS utility operator die alle waarden uitgezonden totdat Observable voltooid is combineert in een enkele array. Dit is ideaal voor situaties waar u de gehele stream als array wilt behandelen, zoals batchverwerking, UI-weergave na batch-acquisitie, en aggregaatverwerking. Omdat het waarden accumuleert tot voltooiing, kan het niet worden gebruikt met oneindige streams.
---

# toArray - Converteer waarden naar array

De `toArray` operator is een operator die **alle waarden uitgezonden door Observable tot voltooiing combineert in een enkele array**.
Dit is nuttig voor batchverwerking, UI-weergave na batch-ophaling, aggregatie, etc.


## 🔰 Basissyntax en werking

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Uitvoer:
// [1, 2, 3]
```

Alle waarden worden gecombineerd in een enkele array, die wordt uitgezonden bij Observable-voltooiing.

[🌐 RxJS Officiële Documentatie - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Typisch gebruiksvoorbeeld

Dit kan worden gebruikt in situaties waar u meerdere asynchrone resultaten in één keer wilt verwerken of in een batch naar de UI wilt uitvoeren.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Ontvang alles bij voltooiing:', result);
  });

// Uitvoer:
// Ontvang alles bij voltooiing: [0, 1, 2, 3, 4]
```


## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Uitvoerweergavegebied
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>toArray voorbeeld:</h3>';
document.body.appendChild(toArrayOutput);

// Individuele waarden weergavegebied
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Individuele waarden:</h4>';
toArrayOutput.appendChild(individualValues);

// Array resultaat weergavegebied
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Array resultaat:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// Abonneer op individuele waarden
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Waarde: ${val}`;
    individualValues.appendChild(valueItem);
  });

// Abonneer op dezelfde stream als array
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Resultaat array: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Toon array-elementen individueel
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Samenvatting

- `toArray` **geeft een array van alle waarden uit bij voltooiing**
- Ideaal voor situaties waar u de gehele stream in aggregaat wilt afhandelen
- Gecombineerd met `concatMap`, `delay`, etc., kan het worden gebruikt voor **asynchrone sequentiële batchverwerking**
