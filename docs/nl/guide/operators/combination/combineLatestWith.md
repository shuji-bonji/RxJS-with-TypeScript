---
description: "combineLatestWith is een RxJS combine operator die de originele Observable combineert met de laatste waarden van andere Observables. Het is ideaal voor situaties waarin je constant de laatste waarden van meerdere afhankelijke stromen wilt controleren, zoals realtime validatie van formulierinvoer, synchronisatie van meerdere toestanden, realtime updates van berekeningsresultaten, etc. De pipeable operator versie is handig voor gebruik binnen pijplijnen."
---

# combineLatestWith - laatste waarden combineren

De `combineLatestWith` operator combineert alle **laatste waarden** van de oorspronkelijke Observable en elke andere gespecificeerde Observable.
Telkens wanneer een nieuwe waarde wordt uitgegeven van een van de Observable, wordt een resultaat uitgegeven dat alle laatste waarden combineert.
Dit is de Pipeable Operator versie van `combineLatest` van Creation Function.

## 🔰 Basissyntaxis en gebruik

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Uitvoervoorbeelden:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Nadat elke Observable **ten minste één waarde** heeft afgegeven, wordt de gecombineerde waarde uitgevoerd.
- Telkens als er een nieuwe waarde aan beide kanten binnenkomt, wordt het laatste paar opnieuw uitgezonden.

[🌐 Officiële RxJS documentatie - ` combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)

## 💡 Typisch gebruikspatroon.

- **Realtime validatie van formulierinvoer**: constante controle van de laatste status van meerdere velden
- **Synchronisatie van meerdere afhankelijke toestanden**: combinatie van configuratiewaarden en gebruikersinvoer
- **Realtime bijwerken van berekeningsresultaten**: onmiddellijke berekening van resultaten van meerdere invoerwaarden

## Praktische codevoorbeelden (met UI)

Voorbeeld van het berekenen van het totaalbedrag in real-time uit prijs- en hoeveelheidsinvoer.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Uitvoergebied maken
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Praktische voorbeelden van:</h3>';
document.body.appendChild(output);

// Invoerveld maken
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Prijs per eenheid';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = 'Hoeveelheid';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// Resultaatweergavegebied
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// van elke invoerObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Berekend door de laatste waarden te combineren
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Totaal bedrag: ¥${total.toLocaleString()}</strong>`;
  });
```

- Wanneer u een van beide velden invoert, wordt het totaal **onmiddellijk herberekend** op basis van de twee meest recente waarden.
- `startWith()` wordt gebruikt om het gecombineerde resultaat vanaf het begin te krijgen.

## Verschillen met de Creation Function `combineLatest`.

### Basisverschillen.

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Uitvoervoorbeelden:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

### Specifieke voorbeelden van gebruik

**Als je alleen eenvoudige combinaties wilt, kun je Creation Function gebruiken**.


```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Eenvoudig en gemakkelijk af te lezen
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}3 (${age}Leeftijd)`);
});
// Uitgang: Yamada Taro (30Leeftijd)
```

**Als je een conversieproces wilt toevoegen aan de hoofdstroom, wordt de Pipeable Operator aanbevolen**.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Zoeken naar...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Alle</option><option>Boeken</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Hoofdstroom: Trefwoorden zoeken
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Na invoer300msWacht
  startWith('')
);

// Substroom: Categorie selectie
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Alle')
);

// ✅ Pipeable OperatorEditie - Compleet in één pijplijn
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // Omgezet naar kleine letters
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Zoeken naar: "${result.term}" Categorie: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation FunctionEditie - Overbodig geworden
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `Zoeken naar: "${result.term}" Categorie: ${result.category} [${result.timestamp}]`;
});
```

**Bij het combineren van meerdere configuratiewaarden**.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Slider maken
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Hoofdstroom: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable OperatorEditie - RedAndere kleuren combineren als hoofdkleur
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### Samenvatting.

- **`combineLatest`**: ideaal als u gewoon meerdere stromen wilt combineren
- **`combineLatestWith`**: ideaal als je de laatste waarden van andere streams wilt combineren tijdens het transformeren of verwerken van de hoofdstroom

## ⚠️ Opmerkingen.

### Wordt pas afgegeven als de beginwaarden beschikbaar zijn.

Er worden geen resultaten uitgevoerd totdat alle Observable ten minste één waarde hebben afgegeven.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Geen waarde afgegevenObservable
).subscribe(console.log);
// Geen uitvoer (omdatNEVERGeen uitvoer (omdat)
```

Dit kan worden opgelost door een beginwaarde op te geven met `startWith()`.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Uitgang: [0, null] → [1, null] → [2, null]
```

### Pas op voor frequente heruitgaven.

Als een stroom vaak waarden afgeeft, wordt het resultaat ook vaak opnieuw uitgegeven.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msStream uitgegeven elke
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$elke keer dat een waarde wordt uitgegeven,fast$wordt uitgegeven, wordt deze gecombineerd met de laatste waarde van
// → Prestaties hebben aandacht nodig
```

### Foutafhandeling.

Als er een fout optreedt in een Observable, eindigt het hele proces met een fout.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Er treden fouten op')).pipe(
      catchError((err: unknown) => of('Herstel'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Uitgang: [0, 'Herstel'] → [1, 'Herstel']
```

## 📚 Verwante operatoren.

- **[combineLatest](/nl/guide/creation-functions/combination/combineLatest)** - versie van de Creation Function.
- **[withLatestFrom](/nl/guide/operators/combination/withLatestFrom)** - alleen geactiveerd door mainstream
- **[zipWith](/nl/guide/operators/combination/zipWith)** - Paart overeenkomstige waarden
