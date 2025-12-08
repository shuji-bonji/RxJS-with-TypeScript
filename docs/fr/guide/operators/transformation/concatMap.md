---
description: "concatMap traite chaque Observable séquentiellement, en attendant que le précédent soit terminé avant de commencer le suivant. Idéal pour les scénarios où l'ordre d'exécution est important, comme les appels API en série ou l'ordre de téléchargement de fichiers garanti. Permet d'obtenir des chaînes de traitement asynchrones sûres avec l'inférence de type TypeScript, et explique les différences avec mergeMap et switchMap."
---

# concatMap - Exécuter chaque Observable séquentiellement

L'opérateur `concatMap` transforme chaque valeur du flux d'entrée en un Observable et **les exécute séquentiellement**.
**Il ne démarre pas l'Observable suivant tant que le précédent n'est pas terminé**.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} terminé`).pipe(delay(1000))
  )
).subscribe(console.log);

// Sortie (dans l'ordre) :
// A terminé
// B terminé
// C terminé
```

- Chaque valeur est transformée en un Observable.
- L'Observable suivant ne s'exécute qu'après la fin de l'Observable précédent.

[🌐 Documentation officielle RxJS - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Modes d'utilisation typiques

- Exécution de requêtes API où l'ordre est important
- Traitement des tâches basé sur une file d'attente
- Animation ou contrôle de l'interface utilisateur étape par étape
- Envoi de messages où l'ordre est important

## 🧠 Exemple de code pratique (avec interface utilisateur)

Exemple où chaque clic sur un bouton déclenche une requête, et les requêtes sont toujours traitées dans l'ordre.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Envoyer requête';
document.body.appendChild(button);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Événement de clic
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Requête ${requestId} démarrée`);
      return of(`Réponse ${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });
```

- Chaque requête est envoyée et terminée dans l'ordre.
- La requête suivante n'est émise que lorsque la précédente est terminée.
