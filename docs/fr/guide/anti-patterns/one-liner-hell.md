---
description: "Explication détaillée de la syntaxe de séparation par étapes pour résoudre l'enfer du one-liner RxJS. En séparant clairement la définition de flux, la transformation et la souscription, et en nommant chaque étape, vous pouvez écrire du code réactif facile à déboguer, tester et lire. Avec des exemples pratiques de refactoring."
---

# Enfer du one-liner et syntaxe de séparation par étapes

La principale cause de l'"enfer du one-liner" dans le code RxJS est que **"définition de flux", "transformation" et "souscription (effets de bord)" sont mélangés**. Cela réduit considérablement la lisibilité et la capacité de débogage.

## Pourquoi l'"enfer du one-liner" se produit-il ?

### ❌ Code problématique courant

```ts
import { fromEvent } from 'rxjs';
import { map, filter, debounceTime, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

fromEvent(document, 'click')
  .pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    debounceTime(300),
    switchMap(x => ajax(`/api?x=${x}`))
  )
  .subscribe(res => {
    if (res.status === 200) {
      console.log('OK');
    } else {
      handleError(res);
    }
  });

function handleError(res: any) {
  console.error('Error:', res);
}
```

### Problèmes

| Problème | Impact |
|---|---|
| **Ligne longue** | Les lecteurs se perdent |
| **Débogage difficile** | Difficile de vérifier l'état intermédiaire |
| **Tests difficiles** | On ne peut que tester le flux entier |
| **Structure de traitement imbriquée** | Les branchements conditionnels dans subscribe deviennent profonds |
| **Non réutilisable** | Le traitement du pipeline ne peut pas être utilisé ailleurs |


## Solution : syntaxe de séparation par étapes (style fonctionnel)

Organisez le code RxJS en "3 étapes aux relations claires".

1. **Définition de flux (source)** - Source des données
2. **Transformation de flux (pipeline)** - Traitement des données
3. **Souscription et effets de bord (subscription)** - Effets de bord comme mise à jour UI et logs


## Pattern recommandé : syntaxe de séparation par étapes

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Définition Observable (source du flux)
const clicks$ = fromEvent(document, 'click');

// 2. Définition pipeline (traitement de transformation des données)
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. Traitement de souscription (exécution des effets de bord)
const subscription = processed$.subscribe({
  next: x => console.log('Position du clic:', x),
  error: err => console.error(err),
  complete: () => console.log('Terminé')
});
```

### Avantages

| Avantage | Détail |
|---|---|
| **Signification claire à chaque étape** | Responsabilité de chaque étape visible d'un coup d'œil |
| **Facile à déboguer** | Peut vérifier les flux intermédiaires avec `console.log` ou `tap` |
| **Facile à tester** | Peut tester individuellement les flux intermédiaires comme `processed$` |
| **Imbrication peu profonde** | Traitement dans subscribe devient simple |
| **Réutilisable** | Peut extraire le traitement du pipeline comme fonction |


## Variation : séparation de fonction (modularisation)

Lorsque le traitement de transformation devient long, **séparez le pipeline en fonction**.

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// Extraire le traitement du pipeline en fonction
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// Côté utilisation
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**Point :** Extraire "comment transformer" en fonction pure **augmente exponentiellement la testabilité**.


## Règles de nommage

Clarifiez l'intention du code par un nommage approprié.

| Étape | Exemples de noms | Signification |
|---|---|---|
| **Source** | `clicks$`, `input$`, `routeParams$` | Source d'événements ou de données |
| **Pipe** | `processed$`, `validInput$`, `apiResponse$` | Flux traité |
| **Subscription** | `subscription`, `uiSubscription` | Effets de bord réellement exécutés |

Le **suffixe `$`** permet de voir d'un coup d'œil que "c'est un Observable".


## Écriture plus déclarative (RxJS 7+)

Extraire `pipe` comme fonction pour la réutilisabilité.

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Définir le pipeline comme fonction (réutilisable)
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**Avantage :** La logique de traitement (`processClicks`) peut être réutilisée dans d'autres flux.


## Before/After : refactoring par pattern typique

Voici des exemples d'amélioration dans des cas d'usage réels.

### A. Événement UI → API → Mise à jour UI

#### ❌ Before (enfer du one-liner)

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, switchMap, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

interface ApiRes {
  items: string[];
  error?: string;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

fromEvent(button, 'click').pipe(
  throttleTime(500),
  switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
  catchError(err => of({ items: [], error: err.message }))
).subscribe(res => {
  list.innerHTML = res.items.map(item => `<li>${item}</li>`).join('');
  if (res.error) alert(res.error);
});
```

#### ✅ After (séparation par étapes + fonctionnalisation)

```ts
import { fromEvent, pipe, of } from 'rxjs';
import { throttleTime, switchMap, map, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface ApiRes {
  items: string[];
}

interface Result {
  items: string[];
  error: string | null;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

// 1) source
const clicks$ = fromEvent(button, 'click');

// 2) pipeline (extrait en fonction pure)
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription (seulement effets de bord)
const subscription = result$.subscribe(({ items, error }) => {
  renderList(items);
  if (error) toast(error);
});

function renderList(items: string[]) {
  list.innerHTML = items.map(item => `<li>${item}</li>`).join('');
}

function toast(message: string) {
  alert(message);
}
```

**Améliorations :**
- Traitement du pipeline `loadItems()` converti en fonction pure
- Effets de bord (`renderList`, `toast`) concentrés côté subscribe
- Facile à tester et déboguer


### B. Valeur de formulaire → Validation → Sauvegarde API (auto-sauvegarde)

#### ❌ Before

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

fromEvent(input, 'input')
  .pipe(
    map((e: Event) => (e.target as HTMLInputElement).value),
    debounceTime(400),
    distinctUntilChanged(),
    filter(v => v.length >= 3),
    switchMap(v => ajax.post('/api/save', { v }))
  )
  .subscribe(
    () => console.log('OK'),
    err => alert(err.message)
  );
```

#### ✅ After (séparation de responsabilités + nommage)

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline (validation)
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline (auto-sauvegarde)
const autosave = () =>
  pipe(
    switchMap((v: string) => ajax.post('/api/save', { v }))
  );

const save$ = value$.pipe(validate(), autosave());

// 3) subscription
const subscription = save$.subscribe({
  next: () => showSuccess(),
  error: (err) => showError(String(err?.message ?? err))
});

function showSuccess() {
  console.log('Sauvegardé');
}

function showError(message: string) {
  alert(message);
}
```

**Améliorations :**
- Validation (`validate`) et sauvegarde (`autosave`) séparées
- Chaque pipeline devient réutilisable
- Tests faciles (peut tester validation et sauvegarde individuellement)


### C. Cache + rafraîchissement manuel

```ts
import { merge, of, Subject } from 'rxjs';
import { switchMap, shareReplay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Item {
  id: number;
  name: string;
}

const refreshBtn = document.getElementById('refresh-btn') as HTMLButtonElement;

// 1) sources
const refresh$ = new Subject<void>();
const initial$ = of(void 0);

// 2) pipeline
const fetchItems$ = merge(initial$, refresh$).pipe(
  switchMap(() => ajax.getJSON<Item[]>('/api/items')),
  shareReplay({ bufferSize: 1, refCount: true }) // Mémorisation
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// Rechargement depuis l'UI
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**Point :**
- Chargement initial automatique (`initial$`) et rafraîchissement manuel (`refresh$`) séparés
- `shareReplay` pour mettre en cache la dernière valeur
- Plusieurs abonnés partagent le même résultat


## Avancé : intégrer des logs intermédiaires

Vous pouvez observer chaque étape avec `tap()`.

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('Clic détecté')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('Coordonnée X:', x))
);

processed$.subscribe(x => console.log('Valeur finale:', x));
```

**Point :**
- `tap` est un opérateur dédié aux effets de bord
- Peut vérifier les valeurs de chaque étape lors du débogage
- Devrait être supprimé en production


## Démonstration de la testabilité

Grâce à la séparation par étapes, **le traitement du pipeline peut être testé individuellement**.

### Exemple : test de validation d'entrée

```ts
// validate.ts
import { pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter } from 'rxjs';

export const validateQuery = () =>
  pipe(
    map((s: string) => s.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((s) => s.length >= 3)
  );
```

```ts
// validate.spec.ts
import { TestScheduler } from 'rxjs/testing';
import { validateQuery } from './validate';

describe('validateQuery', () => {
  it('trims, debounces, distincts, filters length>=3', () => {
    const scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });

    scheduler.run(({ hot, expectObservable }) => {
      // Entrée: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // Attendu: seuls 'abc' et 'abcd' passent
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**Avantage :**
- Peut tester le traitement du pipeline **individuellement**
- Pas de dépendance à DOM/HTTP = **rapide et stable**
- Contrôle de l'axe temporel avec les tests marble

Voir [Méthodes de test](/fr/guide/testing/unit-tests) pour plus de détails.


## Templates d'instructions GitHub Copilot

Collection de prompts utilisables lors du refactoring réel.

### 1. Décomposition en trois étapes

```
Refactoriser ce code RxJS en 3 étapes "source / pipeline / subscription".
Exigences:
- Nommer les Observables avec suffixe $
- Extraire pipeline en fonction retournant pipe(...) (ex: validate(), loadItems())
- Concentrer les effets de bord (mise à jour UI, console, toast) dans subscribe
- Ajouter tap() aux endroits appropriés pour observer l'état intermédiaire (avec commentaires)
- Variables et fonctions avec noms transmettant le domaine
```

### 2. Clarification de la sélection d'opérateur

```
Vouloir prévenir les appels API multiples par clics multiples.
Proposer quel opérateur utiliser parmi switchMap/mergeMap/concatMap/exhaustMap actuel,
et remplacer par le bon opérateur. Écrire la justification en commentaire.

Directives:
- Sauvegarde de formulaire : traitement séquentiel (concatMap)
- Suggestions de recherche : abandonner anciennes requêtes (switchMap)
- Anti-spam de bouton : interdire double exécution (exhaustMap)
```

### 3. Pattern d'auto-sauvegarde

```
Refactoriser le code suivant en pattern d'auto-sauvegarde:
- Entrée avec debounceTime et distinctUntilChanged
- Sauvegarde sérialisée avec concatMap
- Effets de bord pour notifier succès/échec déplacés côté subscribe
- Fonctionnaliser la transformation pour testabilité
- Si possible, mettre en cache l'état récent avec shareReplay
```

### 4. Cache + rafraîchissement manuel

```
Changer en pattern "chargement initial auto + rafraîchissement manuel":
- Introduire refresh$ Subject
- merge(initial$, refresh$) → switchMap(fetch)
- Mettre en cache la dernière valeur avec shareReplay({bufferSize:1, refCount:true})
- Extraire le pipe fetch en fonction pour réutilisation
```


## Conclusion : directives pour écrire lisiblement

| Élément | Contenu recommandé |
|---|---|
| ✅ 1 | **Écrire séparément** Observable, pipe, subscribe |
| ✅ 2 | Flux intermédiaires : **montrer la signification par nom de variable** |
| ✅ 3 | Pipe complexes : **fonctionnaliser** |
| ✅ 4 | **tap() pour vérification intermédiaire** possible |
| ✅ 5 | `processSomething = pipe(...)` pour réutilisabilité |


## Résumé

- **L'enfer du one-liner** se produit par mélange de définition de flux, transformation et souscription
- **Syntaxe de séparation par étapes** (Source → Pipeline → Subscription) pour clarifier les responsabilités
- **Fonctionnalisation du pipeline** améliore testabilité et réutilisabilité
- **Nommage approprié** (suffixe `$`, noms de variables significatifs) améliore lisibilité

## Sections connexes

- **[Erreurs courantes et solutions](/fr/guide/anti-patterns/common-mistakes#13-過度な複雑化)** - Anti-pattern de complexification excessive
- **[Méthodes de test](/fr/guide/testing/unit-tests)** - Comment tester le code RxJS
- **[Comprendre les opérateurs](/fr/guide/operators/)** - Comment utiliser chaque opérateur

## Prochaines étapes

1. Chercher les endroits devenus "enfer du one-liner" dans le code existant
2. Refactoriser avec la syntaxe de séparation par étapes
3. Fonctionnaliser le traitement du pipeline et écrire des tests unitaires
4. Utiliser les templates d'instructions Copilot pour unifier dans toute l'équipe


> [!NOTE]
> Une approche plus complète pour "écrire du RxJS lisible" sera couverte dans le futur **Chapitre 12: Patterns pratiques**.
