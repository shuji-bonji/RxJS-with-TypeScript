---
description: "RxJS-debugtechnieken, van het volgen van waarden met tap(), effectieve plaatsing van console.log, RxJS DevTools-extensie, aangepaste debug-operators, tot prestatiemetingen. Systematische uitleg van praktische debugstrategieën, inclusief het identificeren van problemen waarbij waarden niet stromen."
---

# RxJS-debugtechnieken

Debuggen van RxJS vereist een andere aanpak dan traditionele synchrone debugmethoden, vanwege de asynchrone aard van streams.

Deze pagina biedt basisstrategieën voor het debuggen van RxJS-applicaties en navigatie naar gedetailleerde debugtechnieken.

## Overzicht van debugtechnieken

RxJS-debuggen kan worden onderverdeeld in 4 benaderingen.

| Benadering | Inhoud | Detailpagina |
|----------|------|-----------|
| **Basisstrategieën** | tap-operator, ontwikkelaarstools, RxJS DevTools | Uitgelegd op deze pagina |
| **Veelvoorkomende scenario's** | Waarden stromen niet, geheugenlek, gemiste fouten - 6 typische problemen | [→ Details](/nl/guide/debugging/common-scenarios) |
| **Aangepaste tools** | Named streams, debug-operators, prestatiemetingen | [→ Details](/nl/guide/debugging/custom-tools) |
| **Prestaties** | Monitoring van subscriptions, detectie van herberekeningen, geheugengebruik, best practices | [→ Details](/nl/guide/debugging/performance) |

## Basisdebugstrategieën

### 1. Logging met de `tap`-operator

De `tap`-operator is de meest fundamentele debugtechniek, waarmee je streamwaarden kunt observeren zonder bijwerkingen.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Originele waarde:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 Na map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 Na filter:', value))
  )
  .subscribe(value => console.log('✅ Eindwaarde:', value));

// Output:
// 🔵 Originele waarde: 0
// 🟢 Na map: 0
// 🔵 Originele waarde: 1
// 🟢 Na map: 2
// 🔵 Originele waarde: 2
// 🟢 Na map: 4
// 🔵 Originele waarde: 3
// 🟢 Na map: 6
// 🟡 Na filter: 6
// ✅ Eindwaarde: 6
```

#### Belangrijke punten
- Door `tap` bij elke stap in de pipeline in te voegen, kun je de datastroom volgen
- Gebruik emoji's of labels om de leesbaarheid van logs te verbeteren
- `tap` wijzigt waarden niet, dus je kunt veilig debuglogs invoegen

### 2. Gedetailleerde loginformatie outputten

Voor meer gedetailleerde debuginformatie kun je het Observer-object gebruiken.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Normale stream
of(1, 2, 3)
  .pipe(debug('Normaal'))
  .subscribe();

// Output:
// [Normaal] next: 1
// [Normaal] next: 2
// [Normaal] next: 3
// [Normaal] complete

// Stream met fout
concat(
  of(1, 2),
  throwError(() => new Error('Fout opgetreden'))
)
  .pipe(debug('Fout'))
  .subscribe({
    error: () => {} // Foutafhandeling
  });

// Output:
// [Fout] next: 1
// [Fout] next: 2
// [Fout] error: Error: Fout opgetreden
```

### 3. Verifiëren met ontwikkelaarstools

Debugtechnieken met browser-ontwikkelaarstools.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Helper-functie voor debugging
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Button click-event debuggen
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Verzenden:', data));
}
```

#### Gebruik van ontwikkelaarstools
- Groepeer logs met `console.group()`
- Toon stack traces met `console.trace()`
- Toon arrays en objecten overzichtelijk met `console.table()`
- Plaats breakpoints in `tap`

### 4. RxJS DevTools gebruiken

RxJS DevTools is een debugtool beschikbaar als browserextensie.

#### Installatie
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Belangrijkste functies
- Visualisatie van Observable-subscriptiestatus
- Timelineweergave van streamwaarden
- Detectie van geheugenlekken
- Prestatieanalyse

#### Gebruiksvoorbeeld

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Debug alleen in ontwikkelomgeving inschakelen
// Methode voor het bepalen van omgevingsvariabelen verschilt per buildtool
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Handmatige configuratie: gebruik globale variabele
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Maak observeerbaar voor DevTools
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Gedetailleerde debugtechnieken

Nadat je de basisstrategieën begrijpt, kun je specifieke debugtechnieken leren op de volgende detailpagina's.

### Veelvoorkomende debugscenario's

6 typische problemen die je tegenkomt in de praktijk en hun oplossingen

- Scenario 1: Waarden stromen niet
- Scenario 2: Onverwachte waarden worden geproduceerd
- Scenario 3: Subscription wordt niet voltooid (oneindige stream)
- Scenario 4: Geheugenlek (vergeten unsubscribe)
- Scenario 5: Fouten worden niet opgemerkt
- Scenario 6: Retry-pogingen bijhouden

[→ Bekijk veelvoorkomende debugscenario's](/nl/guide/debugging/common-scenarios)

### Aangepaste debugtools

Hoe je eigen debugtools maakt die passen bij de projectvereisten

- Named stream debugging (tagStream)
- Aangepaste debug-operators maken
- Prestatiemeting-operators (measure)

[→ Bekijk aangepaste debugtools](/nl/guide/debugging/custom-tools)

### Prestatiedebuggen

Applicatie-optimalisatie en best practices

- Controleren en volgen van subscriptions
- Detecteren van onnodige herberekeningen (shareReplay)
- Monitoren van geheugengebruik
- Opzetten van debugomgeving
- Type-veilig debuggen
- Foutgrenzen instellen

[→ Bekijk prestatiedebuggen](/nl/guide/debugging/performance)

## Samenvatting

RxJS-debuggen kan efficiënt worden uitgevoerd door de volgende punten in gedachten te houden.

### Basisstrategieën
- ✅ Observeer elke fase van de stream met de `tap`-operator
- ✅ Gedetailleerde logging met ontwikkelaarstools
- ✅ Visualiseer streams met RxJS DevTools

### Veelvoorkomende scenario's
- ✅ Waarden stromen niet → Controleer vergeten subscription, filtervoorwaarden
- ✅ Onverwachte waarden → Let op operatorvolgorde, delen van referenties
- ✅ Subscription wordt niet voltooid → Gebruik `take` of `takeUntil` voor oneindige streams
- ✅ Geheugenlek → Automatische unsubscribe met `takeUntil`-patroon
- ✅ Gemiste fouten → Implementeer passende foutafhandeling

### Debugtools
- ✅ Flexibel debuggen met aangepaste debug-operators
- ✅ Volg meerdere streams met named streams
- ✅ Identificeer knelpunten met prestatiemetingen

### Prestaties
- ✅ Voorkom geheugenlekken door subscriptions te monitoren
- ✅ Vermijd onnodige herberekeningen met `shareReplay`
- ✅ Controleer regelmatig geheugengebruik

Door deze technieken te combineren, kun je RxJS-applicaties efficiënt debuggen.

## Gerelateerde pagina's

- [Foutafhandeling](/nl/guide/error-handling/strategies) - Strategieën voor foutverwerking
- [Testmethoden](/nl/guide/testing/unit-tests) - Hoe RxJS te testen
- [RxJS anti-patronen](/nl/guide/anti-patterns/) - Veelvoorkomende fouten en oplossingen
- [Pipeline](/nl/guide/operators/pipeline) - Operator-ketens
