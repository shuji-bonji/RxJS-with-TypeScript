---
description: "Anti-patroon preventie checklist voor het schrijven van RxJS code. Bevat 16 best practices voor robuuste en onderhoudbare reactieve code, inclusief juist opzeggen van Subscriptions, correct gebruik van Subject, implementatie van foutafhandeling, en preventie van geheugenlekken."
---

# Anti-patroon Preventie Checklist

Gebruik deze checklist om te controleren of je RxJS code anti-patronen bevat. Klik op elk item voor gedetailleerde uitleg en codevoorbeelden.

## Checklist Items

### 🔴 Vermijd Ernstige Problemen

| Check | Item | Punt |
|:---:|---|---|
| <input type="checkbox" /> | **[Subject publiceren met asObservable()](./common-mistakes#1-subject-extern-blootgesteld)** | Exporteer `Subject` niet direct, maar publiceer als Observable met `asObservable()`<br>Statuswijzigingen alleen mogelijk maken via toegewijde methoden |
| <input type="checkbox" /> | **[Vermijd geneste subscribe](./common-mistakes#2-geneste-subscribe-callback-hell)** | Roep geen andere `subscribe` aan binnen een `subscribe`<br>Maak plat met `switchMap`, `mergeMap`, `concatMap`, etc. |
| <input type="checkbox" /> | **[Oneindige streams altijd opzeggen](./common-mistakes#3-unsubscribe-vergeten-geheugenlek)** | Oneindige streams zoals event listeners altijd opzeggen<br>`takeUntil` patroon of `Subscription` beheer |
| <input type="checkbox" /> | **[shareReplay instellingen expliciet maken](./common-mistakes#4-sharereplay-misbruik)** | Gebruik `shareReplay({ bufferSize: 1, refCount: true })` formaat<br>Activeer reference counting om geheugenlekken te voorkomen |
| <input type="checkbox" /> | **[Vermijd if-statement nesting in subscribe](./subscribe-if-hell)** | Vermijd complexe voorwaardelijke vertakkingen binnen `subscribe` (3 of meer niveaus nesten)<br>Schrijf declaratief met operators zoals `filter`, `iif`, `partition` |

### 🟡 Vermijd Problemen die Aandacht Vereisen

| Check | Item | Punt |
|:---:|---|---|
| <input type="checkbox" /> | **[map is pure functie, bijwerkingen in tap](./common-mistakes#5-bijwerkingen-in-map)** | Wijzig geen status of log output binnen `map`<br>Schei bijwerkingen expliciet af met `tap` operator |
| <input type="checkbox" /> | **[Juiste gebruik van Cold/Hot](./common-mistakes#6-cold-hot-observable-verschil-negeren)** | Converteer HTTP requests etc. naar Hot met `shareReplay`<br>Bepaal of het bij elke subscribe moet worden uitgevoerd of gedeeld |
| <input type="checkbox" /> | **[Converteer Promise met from](./common-mistakes#7-promise-en-observable-ongepast-vermengen)** | Meng Promise en Observable niet<br>Converteer naar Observable met `from()` voor uniforme verwerking |
| <input type="checkbox" /> | **[Beheers hoogfrequente events](./common-mistakes#8-backpressure-negeren)** | Beheers zoekinvoer met `debounceTime`, scrollen met `throttleTime`<br>Elimineer duplicaten met `distinctUntilChanged` |

### 🔵 Verbeter Codekwaliteit

| Check | Item | Punt |
|:---:|---|---|
| <input type="checkbox" /> | **[Behandel fouten correct](./common-mistakes#9-fouten-onderdrukken)** | Vang fouten op met `catchError` en behandel ze correct<br>Toon begrijpelijke foutmeldingen aan gebruikers<br>Probeer opnieuw met `retry` / `retryWhen` indien nodig |
| <input type="checkbox" /> | **[Geef DOM events correct vrij](./common-mistakes#10-dom-event-subscription-lek)** | Zeg subscriptions van `fromEvent` altijd op<br>Automatisch opzeggen bij component vernietiging met `takeUntil` |
| <input type="checkbox" /> | **[Zorg voor type-veiligheid](./common-mistakes#11-gebrek-aan-type-veiligheid-overmatig-gebruik-van-any)** | Definieer interfaces of type aliases<br>Specificeer `Observable<T>` type parameters<br>Maak gebruik van TypeScript type inferentie |
| <input type="checkbox" /> | **[Selecteer juiste operators](./common-mistakes#12-onjuiste-operator-selectie)** | Zoeken: `switchMap`, parallel: `mergeMap`<br>Sequentieel: `concatMap`, dubbelklik preventie: `exhaustMap` |
| <input type="checkbox" /> | **[Eenvoudige verwerking vereist geen RxJS](./common-mistakes#13-overmatige-complexiteit)** | Array verwerking etc. is voldoende met gewone JavaScript<br>Gebruik RxJS voor asynchrone verwerking of event streams |
| <input type="checkbox" /> | **[Beheer status reactief](./common-mistakes#14-status-wijziging-in-subscribe)** | Beheer status met `BehaviorSubject` of `scan`<br>Gebruik `subscribe` als uiteindelijke trigger |
| <input type="checkbox" /> | **[Schrijf tests](./common-mistakes#15-gebrek-aan-tests)** | Voer marble tests uit met `TestScheduler`<br>Maak asynchrone verwerking synchroon testbaar |

## Hoe te Gebruiken

### 1. Bij Code Review

Na het schrijven van nieuwe code, voer een zelf-review uit met deze checklist.

### 2. Bij Pull Requests

Door deze checklist in het pull request template op te nemen, kunnen reviewers en jij controleren met gemeenschappelijke standaarden.

### 3. Periodieke Herziening

Gebruik deze checklist periodiek voor je bestaande codebase om te controleren of er geen anti-patronen zijn binnengeslopen.

### 4. Delen binnen Team

Deel met teamleden en unificeer RxJS best practices.

## Gerelateerde Bronnen

- **[Veelvoorkomende Fouten en Oplossingen](./common-mistakes)** - Gedetailleerde uitleg en codevoorbeelden van elk anti-patroon
- **[Anti-patronen Collectie Top](./index)** - Lijst van anti-patronen en hoe te leren
- **[Foutafhandeling](/nl/guide/error-handling/strategies)** - Best practices voor foutafhandeling
- **[Testmethoden](/nl/guide/testing/unit-tests)** - Hoe RxJS code te testen

## Tips voor Checklist Gebruik

1. **Probeer niet alle items tegelijk perfect te maken**
   - Behandel eerst de ernstige problemen (🔴)
   - Verbeter stapsgewijs

2. **Bepaal prioriteiten binnen het team**
   - Pas het belang aan volgens projectkenmerken
   - Maak een aangepaste checklist

3. **Overweeg automatisering**
   - Automatische controle met statische analysetools zoals ESLint
   - Integreer in CI/CD pipeline

4. **Update regelmatig**
   - Update volgens RxJS versie-upgrades
   - Reflecteer inzichten verkregen uit teamervaring

---

**Belangrijk**: Deze checklist is niet bedoeld om perfecte code te schrijven, maar als gids om veelvoorkomende problemen te vermijden. Gebruik flexibel volgens de context van je project.
