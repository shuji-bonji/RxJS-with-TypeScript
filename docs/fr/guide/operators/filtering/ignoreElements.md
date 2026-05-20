---
description: "L'opérateur ignoreElements est un opérateur de filtrage RxJS qui ignore toutes les valeurs et ne laisse passer que les achèvements et les erreurs. Ceci est utile lorsque l'on attend que le processus se termine."
---

# ignoreElements - seuls les achèvements/erreurs passent

L'opérateur `ignoreElements` **ignore toutes les valeurs** émises par l'Observable source et seules **les notifications d'achèvement et d'erreur** sont transmises en aval.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valeur:', value), // Pas appelé
  complete: () => console.log('Terminé')
});
// Sortie: Terminé
```

**Flux des opérations** :.
1. tous les 1, 2, 3, 4 et 5 sont ignorés
2. seules les notifications d'achèvement sont transmises en aval

[🌐 Official RxJS documentation - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Modèle d'utilisation typique.

- **Attendre l'achèvement du processus** : lorsque vous n'avez pas besoin de la valeur et que vous voulez seulement connaître l'achèvement.
- Exécuter seulement les effets secondaires** : exécuter les effets secondaires avec le tap et ignorer les valeurs.
- Gestion des erreurs** : lorsque vous souhaitez uniquement détecter les erreurs.
- Synchronisation des séquences** : attente de l'achèvement de plusieurs processus

## 🧠 Exemple de code pratique 1 : Attendre la fin du processus d'initialisation

Voici un exemple d'attente de l'achèvement de plusieurs processus d'initialisation.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UICréé
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Initialisation de l'application';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Fonction d'ajout d'un journal d'état
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Processus d'initialisation1: Connexion à la base de données
const initDatabase$ = from(['DBConnexion...', 'Vérification de la table...', 'DBPrêt']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Valeurs ignorées, seul l'achèvement est notifié
);

// Processus d'initialisation2: Fichier de configuration en cours de lecture
const loadConfig$ = from(['Fichier de configuration en cours de lecture...', 'Analyse de la configuration en cours...', 'Application de configuration terminée']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Processus d'initialisation3: Authentification de l'utilisateur
const authenticate$ = from(['Vérification des informations d'authentification en cours...', 'Vérification du jeton en cours...', 'Authentification terminée']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Tous les processus d'initialisation sont exécutés.
addLog('Initialisation commencée...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Tous les processus d'initialisation sont terminés.！L'application peut être lancée.';
    addLog('Application démarrée', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Erreur d'initialisation: ${err.message}`;
  }
});
```

- Un journal détaillé de chaque processus d'initialisation est affiché, mais les valeurs sont ignorées.
- Lorsque tous les processus sont terminés, un message d'achèvement s'affiche.

## 🎯 Exemple de code pratique 2 : Attente de la fin du téléchargement d'un fichier

Cet exemple permet d'afficher la progression du téléchargement de plusieurs fichiers, mais de ne notifier que l'achèvement.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UICréé
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Téléchargement de fichiers';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Chargement démarré';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// Processus de téléchargement de fichiers (avec indication de la progression)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // Les valeurs de progression sont ignorées, seul l'achèvement est notifié
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Tous les fichiers sont téléchargés de manière séquentielle
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Max.23 fichiers en parallèle
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Chargement terminé</strong><br>
        ${files.length}Un fichier a été téléchargé...
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Erreur: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- La progression de chaque fichier est affichée, mais les valeurs de progression elles-mêmes ne sont pas transmises en aval.
- Un message d'achèvement s'affiche lorsque tous les téléchargements sont terminés.

## 🆚 Comparaison avec des opérateurs similaires

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Ignorer toutes les valeurs, l'achèvement est transmis
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('ignoreElements: Terminé')
});
// Sortie: ignoreElements: Terminé

// filter(() => false): Filtrer toutes les valeurs, laisser passer l'achèvement
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('filter: Terminé')
});
// Sortie: filter: Terminé

// take(0): Terminé immédiatement
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('take(0): Terminé')
});
// Sortie: take(0): Terminé
```

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valeur:', value), // Pas appelé
  complete: () => console.log('Terminé')
});
// Sortie: Terminé
```

**Recommandé** : utilisez `ignoreElements()` si vous voulez intentionnellement ignorer toutes les valeurs. L'intention du code sera claire.

## 🔄 Gestion des notifications d'erreur.

`ignoreElements` ignore les valeurs, mais **passe les notifications d'erreur**.


```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Une erreur se produit'))
).pipe(
  ignoreElements()
);

// Cas de réussite
success$.subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('✅ Terminé'),
  error: err => console.error('❌ Erreur:', err.message)
});
// Sortie: ✅ Terminé

// Cas d'erreur
error$.subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('✅ Terminé'),
  error: err => console.error('❌ Erreur:', err.message)
});
// Sortie: ❌ Erreur: Une erreur se produit
```

## ⚠️ Notes.

### 1. les effets secondaires sont exécutés

`ignoreElements` ignore les valeurs, mais les effets de bord (par exemple `tap`) sont exécutés.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Effets secondaires:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('Terminé')
});
// Sortie:
// Effets secondaires: 1
// Effets secondaires: 2
// Effets secondaires: 3
// Terminé
```

### 2. utilisation avec Observable

Lorsqu'il est utilisé avec Infinite Observable, l'abonnement dure éternellement car il n'est jamais terminé.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Mauvais cas: Inachevé
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Terminé') // Pas appelé
});

// ✅ Bon exemple: take Terminé en
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Terminé') // 5Appelé après une seconde
});
```

### 3. les types en TypeScript

La valeur de retour de `ignoreElements` est de type `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements Le résultat de Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value est de type never ce bloc n'est donc pas exécuté
    console.log(value);
  },
  complete: () => console.log('Achèvement seulement')
});
```

### 4. si l'achèvement n'est pas garanti

Si la source ne s'achève pas, l'ignoreElements ne s'achèvera pas non plus.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERne se termine pas et n'émet pas d'erreur
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Terminé') // Pas appelé
});
```

## 💡 Modèles de combinaisons pratiques

### Modèle 1 : Séquence d'initialisation

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valeur:', value), // Pas appelé
  complete: () => console.log('Terminé')
});
// Sortie: Terminé
```

### Schéma 2 : Processus de nettoyage


## 📚 Opérateurs apparentés.

- **[filter](. /filter)** - filtre les valeurs en fonction de conditions.
- **[take](. /take)** - seules les N premières valeurs sont prises en compte.
- **[skip](. /skip)** - les N premières valeurs sont ignorées.
- **[tap](. /utility/tap)** - effectuer une action latérale

## Résumé.

L'opérateur `ignoreElements` ignore toutes les valeurs et ne transmet que les complétions et les erreurs.

- ✅ Idéal lorsque seule une notification d'achèvement est requise.
- Les effets secondaires (Tap) sont exécutés.
- ✅ Les notifications d'erreur sont également transmises
- ✅ L'intention est plus claire que `filter(() => false)`.
- ⚠️ L'Observable infini ne se termine pas
- ⚠️ Le type de valeur de retour est `Observable<never>`.
- ⚠️ La valeur est complètement ignorée, mais les effets de bord sont exécutés.
